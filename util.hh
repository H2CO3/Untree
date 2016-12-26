//
// util.hh
// Untree
//
// Created by Arpad Goretity (H2CO3)
// on 16/12/2016
//
// Licensed under the 2-clause BSD License
//

#ifndef UNTREE_UTIL_HH
#define UNTREE_UTIL_HH

#include <algorithm>
#include <iterator>
#include <vector>
#include <unordered_set>


template<typename Container, typename Element>
static bool contains(const Container &c, const Element &e) {
    return std::find(std::begin(c), std::end(c), e) != std::end(c);
}

template<
    typename Domain,
    typename Fn,
    typename Range = typename std::result_of<Fn(Domain)>::type
>
std::vector<Range> vector_map(const std::vector<Domain> &v, Fn fn) {
    std::vector<Range> result;

    result.reserve(v.size());
    std::transform(v.begin(), v.end(), std::back_inserter(result), fn);

    return result;
}

template<typename Container>
constexpr auto cont_size(const Container &c) noexcept -> decltype(c.size())
{
    return c.size();
}

template<typename T, std::size_t N>
constexpr std::size_t cont_size(const T (&array)[N]) noexcept
{
    return N;
}

template<typename Result, typename Container1, typename Container2>
Result set_union(
    const Container1 &left,
    const Container2 &right
) {
    Result result;

    result.reserve(cont_size(left) + cont_size(right));
    result.insert(std::begin(left),  std::end(left));
    result.insert(std::begin(right), std::end(right));

    return result;
}

template<
    typename Container,
    typename Predicate,
    typename Transform,
    typename Domain = typename Container::value_type,
    typename Range = typename std::result_of<Transform(Domain)>::type
>
std::vector<Range> filter_map(
    const Container &c,
    Predicate predicate,
    Transform xform
) {
    std::vector<Range> result;
    result.reserve(cont_size(c));

    for (const auto &elem : c) {
        if (predicate(elem)) {
            result.push_back(xform(elem));
        }
    }

    return result;
}

#endif // UNTREE_UTIL_HH
