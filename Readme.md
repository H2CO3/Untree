Untree, a visual tree parser
============================

Untree is a program for generating machine-readable, structured
data from a tree depicted in an image.

Usage:
-----
 * `./untree input_image.png`
 * A sample tree is provided in `general_tree.png`.
 * For other images, you will likely need to hand-tune the search parameters in `main()`.

TODO:
----
 * Generate actual tree structure (currently, only an adjacency map is printed)
 * Automatically (programmatically) estimate search parameters based on input image
 * Write up a detailed explanation about the algorithm
 * Make it into a library (involves namespacing/better names in general, etc.)
 * Accept more image formats as input besides PNG, e.g. JPG, ...

License:
-------
The 2-clause BSD License
