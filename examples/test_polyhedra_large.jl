using BenchmarkTools
using LinearAlgebra
using DecompUtil

# seed from which the decomposition starts
pos1 = [0.5,0.5,0.5]

# Generate a random point cloud with two spheres cut out of it
r1 = 0.7
r2 = 1.5
center1 = [1,0.5,0.5]
center2 = [-1,0,0.]
N = 300000
d = 7  # side length of random cube to generate points in
rand_pts = [d*rand(3) .- d/2 for i in 1:N]
obs = [p for p in rand_pts if (norm(p-center1) > r1) && (norm(p-center2) > r2)]
@show length(obs)

# defines the bounding box (i.e., -2:2 on x axis, -2:2 on y axis)
bbox = [2,2,2.]

# define a dilation radius
dilation_radius = 0.1

obs_x = [Float32(o[1]) for o in obs]
obs_y = [Float32(o[2]) for o in obs]
obs_z = [Float32(o[3]) for o in obs]

# precompile
result1 = seedDecomp(pos1, obs, bbox, dilation_radius);
result2 = DecompUtil.seedDecomp_3D_fast(pos1, obs_x, obs_y, obs_z, bbox, dilation_radius);

@btime seedDecomp($pos1, $obs, $bbox, $dilation_radius)
@btime  seedDecomp_3D_fast($pos1, $obs_x, $obs_y, $obs_z, $bbox, $dilation_radius)
