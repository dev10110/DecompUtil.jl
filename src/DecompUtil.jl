module DecompUtil

using LinearAlgebra
using StaticArrays
using DecompUtil_jll


export Hyperplane, Polyhedron, LinearConstraint, LinearConstraints
export seedDecomp


"""
    Hyperplane{N, F}(p, n)

represents a hyperplane passing through the point `p` with normal vector `n`. `N` is the dimension, `F` is the type of the vector. Uses StaticVectors to represent p and n internally.
"""
struct Hyperplane{N, F}
    p::SVector{N, F}
    n::SVector{N, F}
end

function Hyperplane(p, n)
    @assert length(p) == length(n)
    N = length(p)
    p_, n_ = promote(p, n)
    F = eltype(p_)
    sp_ = SVector{N,F}(p_...) 
    sn_ = SVector{N,F}(n_...) 
    return Hyperplane{N, F}(sp_, sn_)
end

"""
    Polyhedron(vs::Vector{Hyperplane})

represents a 2D polyhedron using a set of hyperplanes. Each hyperplane's normal points outside the set.
"""
struct Polyhedron{N, F}
    vs::Vector{Hyperplane{N, F}}
end


"""
    LinearConstraint(a, b)

represents the constraint ``a^T x \\leq b``
"""
struct LinearConstraint{N, F}
    a::SVector{N, F}
    b::F
end

"""
    LinearConstraints(A, b)

represents the linear constraints ``A x \\leq b``
"""
struct LinearConstraints{N, F}
    A::Matrix{F}
    b::Vector{F}
end

function LinearConstraints(A::Matrix{F}, b::Vector{F}) where {F}
    N = size(A, 2)
    return LinearConstraints{N, F}(A, b)
end

"""
  LinearConstraints(P::Polyhedron)

constructs a linear constraints matrix from a polyhedron
"""
function LinearConstraints(P::Polyhedron{N, F}) where {N, F}

    A = zeros(F, length(P.vs), N)
    b = zeros(F, length(P.vs))
    for i = 1:length(P.vs)
        for j=1:N
            A[i, j] = P.vs[i].n[j]
        end
        b[i] = dot(P.vs[i].n, P.vs[i].p)
    end

    return LinearConstraints{N, F}(A, b)
end



"""
    seedDecomp(pos, obs, bbox, dilation_radius, max_poly=1)

Perform a seed decomposition in a 2D or 3D space.

Inputs:
  - `pos` is the starting point of the seed
  - `obs` is a vector of points representing obstacles in the environment. In the format `[[x1, y1], [x2, y2], ....]`. You can pass in a vector of static arrays if you want. 
  - `bbox` is the size of the bounding box within which the decomposition should happen
  - `dilation_radius` is the dilation radius (refer to original paper)
  - `max_poly` is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the library is called again with a larger `max_poly`.

Returns:
  - `Polyhedron` representing the free space.
"""
function seedDecomp(pos, obs, bbox, dilation_radius, max_poly = 1)

    N = length(pos)

    if N == 2
        return seedDecomp_2D(pos, obs, bbox, dilation_radius, max_poly)
    end
    if N==3
        return seedDecomp_3D(pos, obs, bbox, dilation_radius, max_poly)
    end
    
    error("Only the cases of N=2 or 3 have been implemented")


end



"""
    seedDecomp_2D(pos, obs, bbox, dilation_radius, max_poly=1)

Perform a seed decomposition in a 2D space.

Inputs:
  - `pos` is the starting point of the seed
  - `obs` is a vector of points representing obstacles in the environment. In the format `[[x1, y1], [x2, y2], ....]`
  - `bbox` is the size of the bounding box within which the decomposition should happen
  - `dilation_radius` is the dilation radius (refer to original paper)
  - `max_poly` is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the library is called again with a larger `max_poly`.

Returns:
  - `Polyhedron` representing the free space.
"""
function seedDecomp_2D(pos, obs, bbox, dilation_radius, max_poly = 1)

    obs_x = [Float32(o[1]) for o in obs]
    obs_y = [Float32(o[2]) for o in obs]

    nobs = Int32(length(obs_x))

    px = zeros(Float32, max_poly)
    py = zeros(Float32, max_poly)
    nx = zeros(Float32, max_poly)
    ny = zeros(Float32, max_poly)

    n_poly = ccall(
        (:seedDecomp2d_polyhedron, libdecomputil),
        Int32,
        (
            Cfloat,
            Cfloat,  # pos
            Clonglong, # nobs
            Ptr{Cint},
            Ptr{Cint}, #obsx, obsy
            Cfloat,
            Cfloat, # local_bbox
            Cfloat, # dilation radius
            Clonglong, # max_npoly
            Ptr{Cfloat},
            Ptr{Cfloat},
            Ptr{Cfloat},
            Ptr{Cfloat}, #poly p, n
        ),
        Float32(pos[1]),
        Float32(pos[2]),
        nobs,
        obs_x,
        obs_y,
        Float32(bbox[1]),
        Float32(bbox[2]),
        Float32(dilation_radius),
        Int64(max_poly),
        px,
        py,
        nx,
        ny,
    )

    if n_poly > max_poly
        return seedDecomp_2D(pos, obs, bbox, dilation_radius, n_poly)
    end

    # there are enough points, return the polyhedron
    vs = [Hyperplane([px[i], py[i]], [nx[i], ny[i]]) for i = 1:n_poly]
    return Polyhedron(vs)

end

"""
    seedDecomp_3D(pos, obs, bbox, dilation_radius, max_poly=1)

Perform a seed decomposition in a 3D space.

Inputs:
  - `pos` is the starting point of the seed, e.g. `[x0, y0, z0]`
  - `obs` is a vector of points representing obstacles in the environment. In the format `[[x1, y1, z1], [x2, y2, z2], ....]`
  - `bbox` is the size of the bounding box within which the decomposition should happen
  - `dilation_radius` is the dilation radius (refer to original paper)
  - `max_poly` is the assumed maximum number of hyperplanes in the resulting solution. If this is smaller than the true number, the function is called again with a larger `max_poly`.

Returns:
  - `Polyhedron` representing the free space.
"""
function seedDecomp_3D(pos, obs, bbox, dilation_radius, max_poly = 1)

    obs_x = [Float32(o[1]) for o in obs]
    obs_y = [Float32(o[2]) for o in obs]
    obs_z = [Float32(o[3]) for o in obs]

    nobs = Int32(length(obs_x))

    px = zeros(Float32, max_poly)
    py = zeros(Float32, max_poly)
    pz = zeros(Float32, max_poly)
    nx = zeros(Float32, max_poly)
    ny = zeros(Float32, max_poly)
    nz = zeros(Float32, max_poly)

    n_poly = ccall(
        (:seedDecomp3d_polyhedron, libdecomputil),
        Int32,
        (
            Cfloat,
            Cfloat,
            Cfloat,  # pos
            Clonglong, # nobs
            Ptr{Cint},
            Ptr{Cint},
            Ptr{Cint}, #obsx, obsy
            Cfloat,
            Cfloat,
            Cfloat, # local_bbox
            Cfloat, # dilation radius
            Clonglong, # max_npoly
            Ptr{Cfloat},
            Ptr{Cfloat},
            Ptr{Cfloat}, # poly p
            Ptr{Cfloat},
            Ptr{Cfloat},
            Ptr{Cfloat}, #poly n
        ),
        Float32(pos[1]),
        Float32(pos[2]),
        Float32(pos[3]),
        nobs,
        obs_x,
        obs_y,
        obs_z,
        Float32(bbox[1]),
        Float32(bbox[2]),
        Float32(bbox[3]),
        Float32(dilation_radius),
        Int64(max_poly),
        px,
        py,
        pz,
        nx,
        ny,
        nz
    )

    if n_poly > max_poly
        return seedDecomp_3D(pos, obs, bbox, dilation_radius, n_poly)
    end

    # there are enough points, return the polyhedron
    vs = [Hyperplane([px[i], py[i], pz[i]], [nx[i], ny[i], nz[i]]) for i = 1:n_poly]
    return Polyhedron(vs)

end


end # end module DecompUtil
