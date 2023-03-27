```@meta
CurrentModule = DecompUtil
```

# DecompUtil

Documentation for [DecompUtil](https://github.com/dev10110/DecompUtil.jl).

## Quick Start

This library exports only one main thing: `seedDecomp`. This performs a decomposition in a 2D or 3D space, and returns a set of hyperplanes that indicate the safe flight corridor. 

2D example:
```@example main
using DecompUtil

# seed from which the decomposition starts
pos = [0,0.] 

# define all the obstacle points
obs = [[-0.2, 1.5], [1, 0.], [0.8,-1.], [ -0.5, -0.5]]

# defines the bounding box (i.e., -2:2 on x axis, -2:2 on y axis)
bbox = [2,2.]

# I am honestly not too sure what this does,
# and it doesnt seem to have an impact on the result.
# (but it needs to be greater than 0)
dilation_radius = 0.1

result = seedDecomp(pos, obs, bbox, dilation_radius)
```

This returns a vector of `Hyperplane`s. Each hyperplane `v` has a point `v.p` and a normal `v.n` and defines the free area as the halfspace: 
```math
\{ x : n^T x \leq n^T p \}
```

We can convert it to a constraints matrix of the form $$Ax \leq b$$:
```@example main
A, b = constraints_matrix(result)

A
```

```@example main
b
```

and we can verify that the seed lies inside this polyhedron:
```@example main
A * pos - b
```
```@example main
A * pos <= b
```

Since we provide a bounding box, the result is always a bounded polyhedron. 


We can visualize it as follows:
```@example main
using Plots

function plot_2D!(pos, obs, bbox, res)
    
    scatter!([pos[1]], [pos[2]], label="Seed")
    scatter!(first.(obs), last.(obs), label="Obstacles")
    vline!([pos[1] - bbox[1], pos[1] + bbox[1]], label="Bounding Box", linestyle=:dash, linewidth=2, color=:black)
    hline!([pos[2] - bbox[2], pos[2] + bbox[2]], label=false, linestyle=:dash, linewidth=2, color=:black)

    for (i, v) in enumerate(res)
        p = v.p
        n = v.n
        lab = i==1 ? "Hyperplane" : false
        plot!(t -> p[1] + n[2]*t, t-> p[2] - n[1]*t, -4,4, label=lab, color=:blue)
    end

    # plot!(legend = :outerleft)

    
end


plot()
plot_2D!(pos, obs, bbox, result)
xlims!(-2.5, 2.5)
ylims!(-2.5, 2.5)
plot!(aspect_ratio=:equal)
```

Finally, for convenience, a `shrink` function is provided, that shrinks the polyhedron by a distance `d`

```@example main

shrunk_result = DecompUtil.shrink(result, 0.3)

plot()
plot_2D!(pos, obs, bbox, shrunk_result)
xlims!(-2.5, 2.5)
ylims!(-2.5, 2.5)
plot!(aspect_ratio=:equal)
```

Note, there is no guarantee that the shrunk polyhedron contains the seed, or indeed is non-empty. Also, note the polyhedron can have redundant constraints.  

Note, no plotting utilities are not exported, so as to not introduce dependencies on plotting. 



## Reference

```@index
```

```@autodocs
Modules = [DecompUtil]
```
