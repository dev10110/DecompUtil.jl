using DecompUtil
using Test

@testset "DecompUtil.jl - 2D Seed Decomposition" begin
    

    pos = [0,0.]
    obs = [[-0.2, 1.5], [1, 0.], [0.8,-1.], [ -0.5, -0.5]]
    bbox = [2,2.]

    res = DecompUtil.seedDecomp_2D(pos, obs, bbox, 0.1)

    println(res)

    @test length(res.vs) == 8

    cons = DecompUtil.constraints(res)

    println(cons.A)

    println(cons.b)

end
