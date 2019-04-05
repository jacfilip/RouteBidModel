using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using RouteBidModel

include("../src/decls.jl")

g =  SimpleWeightedDiGraph(5)
add_edge!(g, 2, 3, 730.0)
add_edge!(g, 3, 1, 910.0)
add_edge!(g, 2, 1, 110.0)
add_edge!(g, 1, 3, 100.0)
add_edge!(g, 4, 1, 700.0)
add_edge!(g, 5, 4, 330.0)

nw = Decls.Network(g)
Decls.InitNetwork!(nw)
Decls.SetSpawnsAndDests!(nw, [3], [5])
for i in 1:10
    Decls.SpawnAgentAtRandom(nw)
end
