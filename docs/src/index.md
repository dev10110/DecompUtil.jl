```@meta
CurrentModule = DecompUtil
```

# DecompUtil

Documentation for [DecompUtil](https://github.com/dev10110/DecompUtil.jl).

This is a julia wrapper to [DecompUtil](https://github.com/sikang/DecompUtil), created by Devansh Agrawal. I have added a few small functionalities, but have not exported all of the functions provided by the original authors. If there are some particular functions you would like to be able to use, let me know and I should be able to wrap them without much difficulty. 

This is research code, and so there is no guarantee of correctness, but so far it has worked well for me. If you find an error, please let me know by raising a Github Issue. 

All credits should go to the original authors:
```
@article{liu2017planning,
  title={Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments},
  author={Liu, Sikang and Watterson, Michael and Mohta, Kartik and Sun, Ke and Bhattacharya, Subhrajit and Taylor, Camillo J and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters},
  volume={2},
  number={3},
  pages={1688--1695},
  year={2017},
  publisher={IEEE}
}
```

For those interested, the wrapper works by exposing a [`C` interface](https://github.com/dev10110/DecompUtil_C) to the original library, creating a [JLL package](https://github.com/JuliaBinaryWrappers/DecompUtil_jll.jl) using [BinaryBuilder](https://github.com/JuliaPackaging/BinaryBuilder.jl), and then creating the [Julia package](https://github.com/dev10110/DecompUtil.jl).


## Quick Start

The main export of this library is `seedDecomp`. This performs a decomposition in a 2D or 3D space, and returns a set of hyperplanes that indicate the safe flight corridor. 

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
