//
// untree.cc
// Untree
//
// Created by Arpad Goretity (H2CO3)
// on 16/12/2016
//
// Licensed under the 2-clause BSD License
//

#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <cstdio>
#include <cmath>
#include <climits>
#include <cassert>
#include <cinttypes>

#include "imgproc.hh"
#include "util.hh"


struct SearchParams {
    int fanout_count_diameter;
    int blur_radius;
    std::uint16_t black_threshold;
    std::uint16_t gray_threshold;
};


static unsigned num_fanouts(
    const GrayscaleImage &img,
    const SearchParams &params,
    Index index
) {
    const auto black_threshold = params.black_threshold;
    const auto diameter = params.fanout_count_diameter;

    // only consider points that are actually part of the line,
    // i.e. those that are black.
    if (img[index] >= black_threshold) {
        return 0;
    }

    const auto x0 = index.x;
    const auto y0 = index.y;

    unsigned n = 0;

    // boundaries of the square of which the contour
    // is used for counting the number of fanouts at (x0, y0)
    std::ptrdiff_t x1 = x0 - diameter / 2;
    std::ptrdiff_t x2 = x0 + diameter / 2;
    std::ptrdiff_t y1 = y0 - diameter / 2;
    std::ptrdiff_t y2 = y0 + diameter / 2;

    std::vector<Index> square_contour;
    square_contour.reserve(4 * diameter + 8);

    // enumerate the pixels of the lower, right, top and left
    // sides of the square, in positive (CCW) direction.
    for (auto x = x1; x < x2; x++) {
        square_contour.push_back({ x, y1 });
    }
    for (auto y = y1; y < y2; y++) {
        square_contour.push_back({ x2, y });
    }
    for (auto x = x2; x > x1; x--) {
        square_contour.push_back({ x, y2 });
    }
    for (auto y = y2; y > y1; y--) {
        square_contour.push_back({ x1, y });
    }

    // '<=' is used so that we go around the square in a full,
    // closed loop, of which the beginning and the end overlap.
    for (std::size_t i = 0; i <= /* ! */ square_contour.size(); i++) {
        auto prev_idx = square_contour[(i + 0) % square_contour.size()];
        auto next_idx = square_contour[(i + 1) % square_contour.size()];

        auto prev_x = prev_idx.x;
        auto prev_y = prev_idx.y;

        auto next_x = next_idx.x;
        auto next_y = next_idx.y;

        // ignore out-of-bounds (non-)pixels
        if (prev_x < 0 || prev_x >= img.width || prev_y < 0 || prev_y >= img.height
         || next_x < 0 || next_x >= img.width || next_y < 0 || next_y >= img.height) {
            continue;
        }

        // count the trailing edge of each radial 'line'
        bool prev_b = img[prev_idx] < black_threshold;
        bool next_b = img[next_idx] < black_threshold;

        if (prev_b == false && next_b == true) {
            n++;
        }
    }

    return n;
}

// The binarized image will be black where the predicate is true
template<typename Fn>
static GrayscaleImage binarized_image(const GrayscaleImage &img, Fn predicate) {
    // make all-black image
    GrayscaleImage tmp(img.width, img.height);

    for (std::ptrdiff_t x = 0; x < img.width; x++) {
        for (std::ptrdiff_t y = 0; y < img.height; y++) {
            // where the predicate is false, make it white
            if (not predicate(Index { x, y })) {
                tmp[{x, y}] = UINT16_MAX;
            }
        }
    }

    return tmp;
}

// the same as binarized_image, but in-place, mutating
template<typename Fn>
static void binarize_image(GrayscaleImage &img, Fn predicate) {
    for (std::ptrdiff_t x = 0; x < img.width; x++) {
        for (std::ptrdiff_t y = 0; y < img.height; y++) {
            img[{x, y}] = predicate(Index { x, y }) ? 0 : UINT16_MAX;
        }
    }
}

static GrayscaleImage image_with_marked_branches(
    const GrayscaleImage &img,
    const SearchParams &params
) {
    // Mark pixels from where one can observe a binary branch (fanout == 3) as black
    auto tmp = binarized_image(img, [&](auto index) {
        return num_fanouts(img, params, index) == 3;
    });

    // Blur the image so that 1-pixel noise is filtered out,
    // and nearby islands are merged. Both effects are
    // needed for preventing false positives (i.e., observing
    // a branch where there isn't one).
    tmp.box_blur(params.blur_radius);

    // Mark significantly dark pixels as black in the blurred image
    binarize_image(tmp, [&](auto index) {
        return tmp[index] < params.gray_threshold;
    });

    return tmp;
}

static GrayscaleImage image_with_marked_leaves(
    const GrayscaleImage &img,
    const SearchParams &params
) {
    // Mark pixels from where one can observe a leaf (fanout == 1) as black
    return binarized_image(img, [&](auto index) {
        return num_fanouts(img, params, index) == 1;
    });
}

// Performs a flood fill on the binary image starting at 'start_index'.
// 'start_index' must point inside a contiguous black island.
// For efficiency and simplicity, this destroys the black island
// (i.e. it replaces it with all white pixels).
// visitor_callback must return bool. If it returns false,
// filling is aborted.
template<typename Fn>
static void flood_fill_binary_image(
    GrayscaleImage &img,
    Index start_index,
    Fn visitor_callback
) {
    assert(img[start_index] == 0 && "pixel at the starting index must be black");

    std::queue<Index> indices_to_visit;

    // Visits the pixel at coordinates 'idx'
    auto visit = [&](Index index) {
        if (visitor_callback(index)) {
            indices_to_visit.push(index);
            img[index] = UINT16_MAX; // white
            return true;
        }
        return false;
    };

    if (not visit(start_index)) {
        return;
    }

    while (not indices_to_visit.empty()) {
        // extract next index to visit
        auto index = indices_to_visit.front();
        indices_to_visit.pop();

        // push all unvisited neighbors that are in-bounds
        auto x = index.x;
        auto y = index.y;

        const Index neighbors[] = {
            { x - 1, y     }, // left
            { x + 1, y     }, // right
            { x,     y + 1 }, // top
            { x,     y - 1 }, // bottom
            { x - 1, y - 1 }, // bottom left
            { x + 1, y - 1 }, // bottom right
            { x - 1, y + 1 }, // top left
            { x + 1, y + 1 }, // top right
        };

        for (auto neigh_idx : neighbors) {
            // ignore OOB neighbors
            if (
                neigh_idx.x < 0 || neigh_idx.x >= img.width
                ||
                neigh_idx.y < 0 || neigh_idx.y >= img.height
            ) {
                continue;
            }

            // is it black and still unvisited?
            // If so, visit it.
            if (img[neigh_idx] == 0) {
                if (not visit(neigh_idx)) {
                    return;
                }
            }
        }
    }
}


using IndexSet = std::vector<Index>;

// This modifies the image: erases the black island at start_index
// (it will become all white)
static IndexSet indices_of_black_island_by_erasing(
    GrayscaleImage &img,
    Index start_index
) {
    // this accumulates the indices of black pixels we encounter
    IndexSet visited_indices;
    visited_indices.reserve(512); // hopefully big enough for most cases...

    flood_fill_binary_image(img, start_index, [&](auto index) {
        visited_indices.push_back(index);
        return true;
    });

    return visited_indices;
}

// The following function returns the indices of pixels in each contiguous
// black (pixel == 0) area, grouped by area.
// The input should be a binarized (black and white only) image.
// For efficiency, this destroys the islands (replaces them with white).
static std::vector<IndexSet> indices_of_each_black_island_by_erasing(GrayscaleImage &img) {
    std::vector<IndexSet> index_sets;
    index_sets.reserve(256);

    for (std::ptrdiff_t x = 0; x < img.width; x++) {
        for (std::ptrdiff_t y = 0; y < img.height; y++) {
            // find a black pixel
            if (img[{x, y}] == 0) {
                auto indices = indices_of_black_island_by_erasing(img, {x, y});
                index_sets.push_back(std::move(indices));
            }
        }
    }

    return index_sets;
}

static Index index_by_collapsing_island(
    const GrayscaleImage &img,
    const IndexSet &island
) {
    // we just want to find one pixel in the intersection of the tree and the island
    return *std::find_if(island.begin(), island.end(), [&](auto index) {
        // is the pixel black (== part of the tree) in the original image?
        return img[index] == 0;
    });
}

static std::vector<Index> indices_by_collapsing_islands(
    const GrayscaleImage &img,
    const std::vector<IndexSet> &islands
) {
    return vector_map(
        islands,
        [&](const auto &island) { return index_by_collapsing_island(img, island); }
    );
}

using AdjacencyMap = std::unordered_map<Index, std::unordered_set<Index>>;

static AdjacencyMap adjacency_map_from_points(
    const GrayscaleImage &input_img,
    const std::vector<Index> &branches,
    const std::vector<Index> &leaves
) {
    const auto points = set_union<std::unordered_set<Index>>(branches, leaves);

    // this map contains the number of outgoing/incoming edges of each
    // node. When an edge E between two nodes (N1, N2) is traversed, it is
    // 'erased', so both nodes will have their neighbor (edge) count decremented.
    std::unordered_map<Index, unsigned> unvisited_neighbor_counts;

    // actually fill the map based on the initial state of the tree
    unvisited_neighbor_counts.reserve(branches.size() + leaves.size());
    for (auto index : branches) {
        unvisited_neighbor_counts[index] = 3;
    }
    for (auto index : leaves) {
        unvisited_neighbor_counts[index] = 1;
    }

    // set of coordinates already visited by flood fill
    std::unordered_set<Index> visited_indices;
    visited_indices.reserve(points.size());

    AdjacencyMap adjacency_map;
    adjacency_map.reserve(points.size());

    // Spin until all nodes have been visited.
    // (In a tree of N nodes, there are N - 1 edges.)
    while (visited_indices.size() < points.size() - 1) {
        // Array ('set') of coordinates where flood fill starts.
        // We want to start at current leaves --- nodes that
        // have only one adjacent neighbor.
        // We must also avoid modifying the unvisited_neighbor_counts map
        // while iterating over it, hence this copy.
        const auto start_indices = filter_map(
            unvisited_neighbor_counts,
            [](auto key_value) { return key_value.second == 1; }, // predicate
            [](auto key_value) { return key_value.first; }        // project to key
        );

        // flood fill starting from the leaves we just enumerated
        for (auto start_index : start_indices) {
            auto img = input_img;

            flood_fill_binary_image(img, start_index, [&](auto index) {
                if (
                    index != start_index // a node is not a neighbor of itself
                    &&
                    contains(points, index) // the coordinate must be part of a node
                    &&
                    not contains(visited_indices, index) // only traverse each node once
                ) {
                    // insert into adjacency map symmetrically
                    bool ok1, ok2;
                    std::tie(std::ignore, ok1) = adjacency_map[index].insert(start_index);
                    std::tie(std::ignore, ok2) = adjacency_map[start_index].insert(index);
                    assert(ok1 && ok2 && "duplicate node found");

                    // mark node as already visited
                    visited_indices.insert(start_index);

                    // register the fact that the edge between the two
                    // nodes 'start_index' and 'index' has been 'erased'
                    unvisited_neighbor_counts[start_index]--;
                    unvisited_neighbor_counts[index]--;

                    #ifndef NDEBUG
                    std::printf("%s <-> %s\n", start_index.to_string().c_str(), index.to_string().c_str());
                    #endif

                    // abort flood fill
                    return false;
                }
                return true;
            });
        }
    }

    return adjacency_map;
}


struct TreeNode {
    Index index;
    std::array<std::unique_ptr<TreeNode>, 2> children;

    TreeNode(Index p_index) : index(p_index) {}

    TreeNode *left() {
        return children[0].get();
    }

    TreeNode *right() {
        return children[1].get();
    }

    const TreeNode *left() const {
        return children[0].get();
    }

    const TreeNode *right() const {
        return children[1].get();
    }

    void add_child(std::unique_ptr<TreeNode> child) {
        // find first empty slot
        auto it = std::find(children.begin(), children.end(), nullptr);
        assert(it != children.end() && "no room left for adding child nodes");
        *it = std::move(child);
    }

    template<typename Fn>
    void traverse(Fn callback) const {
        traverse(callback, 0);
    }

    template<typename Fn>
    void traverse(Fn callback, unsigned depth) const {
        callback(index, depth);

        for (auto &child : children) {
            if (child) {
                child->traverse(callback, depth + 1);
            }
        }
    }
};

// This destroys the adjacency map by erasing each
// edge after it has been traversed.
std::unique_ptr<TreeNode> tree_from_adjacency_map(
    AdjacencyMap &adj_map,
    Index root
) {
    auto node = std::make_unique<TreeNode>(root);

    for (auto it = adj_map[root].begin(); it != adj_map[root].end(); ) {
        auto neighbor = *it;

        it = adj_map[root].erase(it);
        adj_map[neighbor].erase(root);

        node->add_child(tree_from_adjacency_map(adj_map, neighbor));
    }

    return node;
}

// Returns the tree that is recognized in the image
static std::unique_ptr<TreeNode> tree_from_image(
    const GrayscaleImage &img,
    const SearchParams &params
) {
    auto branch_islands = image_with_marked_branches(img, params);
    const auto branch_island_indices = indices_of_each_black_island_by_erasing(branch_islands);
    const auto branches = indices_by_collapsing_islands(img, branch_island_indices);

    auto leaf_islands = image_with_marked_leaves(img, params);
    const auto leaf_island_indices = indices_of_each_black_island_by_erasing(leaf_islands);
    const auto leaves = indices_by_collapsing_islands(img, leaf_island_indices);

    assert(branches.size() + 2 == leaves.size() && "input is not a binary tree");

    #ifndef NDEBUG
    std::printf("# of branches: %zu\n", branches.size());
    std::printf("# of leaves:   %zu\n", leaves.size());
    #endif

    auto adj_map = adjacency_map_from_points(img, branches, leaves);

    return tree_from_adjacency_map(adj_map, leaves[0]);
}

int main(int argc, char *argv[]) {
    const GrayscaleImage img(argv[1]);

    const SearchParams params {
        18,
        7,
        32768,
        47000,
    };

    const auto tree = tree_from_image(img, params);

    std::printf("\n");

    tree->traverse([](Index index, unsigned depth) {
        auto str = index.to_string();
        std::printf("%*s\n", int(str.size() + 2 * depth), str.c_str());
    });

    return 0;
}
