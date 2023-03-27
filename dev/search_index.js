var documenterSearchIndex = {"docs":
[{"location":"","page":"Home","title":"Home","text":"CurrentModule = DecompUtil","category":"page"},{"location":"#DecompUtil","page":"Home","title":"DecompUtil","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"Documentation for DecompUtil.","category":"page"},{"location":"","page":"Home","title":"Home","text":"","category":"page"},{"location":"","page":"Home","title":"Home","text":"Modules = [DecompUtil]","category":"page"},{"location":"#DecompUtil.Hyperplane","page":"Home","title":"DecompUtil.Hyperplane","text":"Hyperplane{N, F}(p, n)\n\nrepresents a hyperplane passing through the point p with normal vector n. N is the dimension, F is the type of the vector. Uses StaticVectors to represent p and n internally.\n\n\n\n\n\n","category":"type"},{"location":"#DecompUtil.LinearConstraint","page":"Home","title":"DecompUtil.LinearConstraint","text":"LinearConstraint(a, b)\n\nrepresents the constraint a^T x leq b\n\n\n\n\n\n","category":"type"},{"location":"#DecompUtil.LinearConstraints","page":"Home","title":"DecompUtil.LinearConstraints","text":"LinearConstraints(A, b)\n\nrepresents the linear constraints A x leq b\n\n\n\n\n\n","category":"type"},{"location":"#DecompUtil.LinearConstraints-Union{Tuple{Polyhedron{N, F}}, Tuple{F}, Tuple{N}} where {N, F}","page":"Home","title":"DecompUtil.LinearConstraints","text":"LinearConstraints(P::Polyhedron)\n\nconstructs a linear constraints matrix from a polyhedron\n\n\n\n\n\n","category":"method"},{"location":"#DecompUtil.Polyhedron","page":"Home","title":"DecompUtil.Polyhedron","text":"Polyhedron(vs::Vector{Hyperplane})\n\nrepresents a 2D polyhedron using a set of hyperplanes. Each hyperplane's normal points outside the set.\n\n\n\n\n\n","category":"type"},{"location":"#DecompUtil.seedDecomp","page":"Home","title":"DecompUtil.seedDecomp","text":"seedDecomp(pos, obs, bbox, dilation_radius, max_poly=1)\n\nPerform a seed decomposition in a 2D or 3D space.\n\nInputs:\n\npos is the starting point of the seed\nobs is a vector of points representing obstacles in the environment. In the format [[x1, y1], [x2, y2], ....]. You can pass in a vector of static arrays if you want. \nbbox is the size of the bounding box within which the decomposition should happen\ndilation_radius is the dilation radius (refer to original paper)\nmax_poly is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the library is called again with a larger max_poly.\n\nReturns:\n\nPolyhedron representing the free space.\n\n\n\n\n\n","category":"function"},{"location":"#DecompUtil.seedDecomp_2D","page":"Home","title":"DecompUtil.seedDecomp_2D","text":"seedDecomp_2D(pos, obs, bbox, dilation_radius, max_poly=1)\n\nPerform a seed decomposition in a 2D space.\n\nInputs:\n\npos is the starting point of the seed\nobs is a vector of points representing obstacles in the environment. In the format [[x1, y1], [x2, y2], ....]\nbbox is the size of the bounding box within which the decomposition should happen\ndilation_radius is the dilation radius (refer to original paper)\nmax_poly is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the library is called again with a larger max_poly.\n\nReturns:\n\nPolyhedron representing the free space.\n\n\n\n\n\n","category":"function"},{"location":"#DecompUtil.seedDecomp_3D","page":"Home","title":"DecompUtil.seedDecomp_3D","text":"seedDecomp_3D(pos, obs, bbox, dilation_radius, max_poly=1)\n\nPerform a seed decomposition in a 3D space.\n\nInputs:\n\npos is the starting point of the seed, e.g. [x0, y0, z0]\nobs is a vector of points representing obstacles in the environment. In the format [[x1, y1, z1], [x2, y2, z2], ....]\nbbox is the size of the bounding box within which the decomposition should happen\ndilation_radius is the dilation radius (refer to original paper)\nmax_poly is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the function is called again with a larger max_poly.\n\nReturns:\n\nPolyhedron representing the free space.\n\n\n\n\n\n","category":"function"}]
}