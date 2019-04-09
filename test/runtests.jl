using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Compose

include("../src/decls.jl")
include("../src/Visuals.jl")

#using .Decls
#using .Visuals

g =  SimpleWeightedDiGraph(5)
add_edge!(g, 2, 3, 730.0)
add_edge!(g, 3, 1, 910.0)
add_edge!(g, 2, 1, 110.0)
add_edge!(g, 5, 2, 230.0)
add_edge!(g, 1, 3, 100.0)
add_edge!(g, 4, 1, 700.0)
add_edge!(g, 5, 4, 330.0)

nw = Decls.Network(g)
Decls.SetLL(nw, [100., 600., 100., 600., 700.], [100., 100., 600., 600., 400.])
Decls.SetSpawnsAndDests!(nw, [3], [5])
for i in 1:2
    Decls.SpawnAgentAtRandom(nw)
end

sim = Decls.Simulation(nw, 60, 1)
Decls.RunSim(sim)

Visuals.test2(comp)

v = @linq sim.simData |> where(:agent .==  1) |> select(:lat, :lon)

comp = Visuals.DrawGraph(Decls.GetIntersectionCoords(nw))
comp2 = Visuals.MarkPositions(v[:,1], v[:,2])

compose(comp, comp2)

ncoords = Decls.GetIntersectionCoords(nw)
