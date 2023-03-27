using DecompUtil
using Test

@testset "DecompUtil.jl - 2D Seed Decomposition" begin
    

    pos = [0,0.]
    obs = [[-0.2, 1.5], [1, 0.], [0.8,-1.], [ -0.5, -0.5]]
    bbox = [2,2.]

    res = seedDecomp(pos, obs, bbox, 0.1)


    @test length(res.vs) == 8

    cons = LinearConstraints(res)

    @test size(cons.A) == (8, 2) 
    @test size(cons.b) == (8,) 


end

@testset "DecompUtil.jl - 3D Seed Decomposition" begin
    

    pos = [0,0., 0.0]
    obs = [[-0.2, 1.5, 1], [1, 0., -1], [0.8,-1., 1], [ -0.5, -0.5, -1]]
    bbox = [2,2., 2.]

    res = seedDecomp(pos, obs, bbox, 0.1)


    @test length(res.vs) == 10

    cons = LinearConstraints(res)

    @test size(cons.A) == (10, 3)
    @test size(cons.b) == (10, )

end
