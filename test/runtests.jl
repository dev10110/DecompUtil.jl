using DecompUtil
using LinearAlgebra
using Test

@testset "DecompUtil.jl - 2D Seed Decomposition" begin
    

    pos = [0,0.]
    obs = [[-0.2, 1.5], [1, 0.], [0.8,-1.], [ -0.5, -0.5]]
    bbox = [2,2.]

    res = seedDecomp(pos, obs, bbox, 0.1)

    @test length(res) == 8

    A, b = constraints_matrix(res)

    @test size(A) == (8, 2) 
    @test size(b) == (8,) 

    @test any(isnan.(A)) == false
    @test any(isnan.(b)) == false


end

@testset "DecompUtil.jl - 3D Seed Decomposition" begin
    

    pos = [0,0., 0.0]
    obs = [[-0.2, 1.5, 1], [1, 0., -1], [0.8,-1., 1], [ -0.5, -0.5, -1]]
    bbox = [2,2., 2.]

    res = seedDecomp(pos, obs, bbox, 0.1)


    @test length(res) == 10

    A, b = constraints_matrix(res)

    @test size(A) == (10, 3)
    @test size(b) == (10, )
    
    @test any(isnan.(A)) == false
    @test any(isnan.(b)) == false

end

@testset "DecompUtil.jl - 3D Seed Decomposition, Fast" begin
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

    # defines the bounding box (i.e., -2:2 on x axis, -2:2 on y axis)
    bbox = [2,2,2.]

    # define a dilation radius
    dilation_radius = 0.1

    obs_x = [Float32(o[1]) for o in obs]
    obs_y = [Float32(o[2]) for o in obs]
    obs_z = [Float32(o[3]) for o in obs]

    res = DecompUtil.seedDecomp_3D_fast(pos1, obs_x, obs_y, obs_z, bbox, dilation_radius);

    @test length(res) > 1

end
