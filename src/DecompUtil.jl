module DecompUtil

using LinearAlgebra
using DecompUtil_jll


"""
    Hyperplane_2D(p, n)

represents a hyperplane passing through the point `p` with normal vector `n`
"""
struct Hyperplane_2D{F}
  p::Vector{F}
  n::Vector{F}
end

"""
    Polyhedron_2D(vs::Vector{Hyperplane_2D})

represents a 2D polyhedron using a set of hyperplanes. Each hyperplane's normal points outside the set.
"""
struct Polyhedron_2D{F}
  vs::Vector{Hyperplane_2D{F}}
end


"""
    LinearConstraint(a, b)

represents the constraint `a^T x \\leq b`
"""
struct LinearConstraint{F}
  a::Vector{F}
  b::F
end

"""
    LinearConstraints(A, b)

represents the constraint `A x \\leq b`
"""
struct LinearConstraints{F}
  A::Matrix{F}
  b::Vector{F}
end

function constraints(P::Polyhedron_2D{F}) where {F}

  A = zeros(F, length(P.vs), 2)
  b = zeros(F, length(P.vs))
  for i=1:length(P.vs)
    A[i, 1] = P.vs[i].n[1]
    A[i, 2] = P.vs[i].n[2]
    b[i]    = dot(P.vs[i].n, P.vs[i].p)
  end

  return LinearConstraints(A, b)
end
    


function seedDecomp_2D(pos, obs, bbox, dilation_radius, max_poly=1)

  obs_x = [Float32(o[1]) for o in obs]
        obs_y = [Float32(o[2]) for o in obs]

        nobs = Int32(length(obs_x))

        px = zeros(Float32, max_poly)
        py = zeros(Float32, max_poly)
        nx = zeros(Float32, max_poly)
        ny = zeros(Float32, max_poly)

        n_poly = ccall( (:seedDecomp2d_polyhedron, libdecomputil),
                       Int32,
                       (Cfloat, Cfloat,  # pos
                                Clonglong, # nobs
                                Ptr{Cint}, Ptr{Cint}, #obsx, obsy
                                Cfloat, Cfloat, # local_bbox
                                Cfloat, # dilation radius
                                Clonglong, # max_npoly
                                Ptr{Cfloat}, Ptr{Cfloat},Ptr{Cfloat},Ptr{Cfloat}, #poly p, n
                        ),
                       Float32(pos[1]), Float32(pos[2]),
                       nobs,
                       obs_x, obs_y,
                       Float32(bbox[1]), Float32(bbox[2]),
                       Float32(dilation_radius),
                       Int64(max_poly),
                       px, py, nx, ny
                       )

        if n_poly > max_poly
                return seedDecomp_2D(pos, obs, bbox, dilation_radius, n_poly)
        end

        # there are enough points, return the polyhedron
        vs = [Hyperplane_2D([px[i], py[i]], [nx[i], ny[i]]) for i=1:n_poly]
        return Polyhedron_2D(vs)

end


end # end module DecompUtil
