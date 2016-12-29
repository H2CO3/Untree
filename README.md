Untree, a visual tree parser
============================

Untree is a program for generating machine-readable, structured
data from a tree depicted in an image.

Explanation
-----------
A detailed explanation of the algorithm can be found on [my blog](http://h2co3.org/blog/index.php/2016/12/26/untree-or-i-will-get-my-data-anyway/).

Usage:
-----
 * `./untree input_image.png`
 * A sample tree is provided in `general_tree.png`.
 * For other images, you will likely need to hand-tune the search parameters in `main()`.

Dependencies:
------------
 * `C++14`
 * `libpng ≥ 1.6`

TODO:
----
 * Automatically (programmatically) estimate search parameters based on input image
 * Preprocess arbitrary images of trees in order to obtain a nice, binarized image
   that only contains pixels from the tree structure itself (no captions, etc.)
 * OCR description texts and associate them with leaves
 * Allow users to specify which node should be the root
 * Make it into a library (involves namespacing/better names in general, etc.)
 * Accept more image formats as input besides PNG, e.g. JPG, ...

License:
-------
The 2-clause BSD License
