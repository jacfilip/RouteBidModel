using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")
Pkg.add("DataFrames")
Pkg.add("DataFramesMeta")
Pkg.add("Distributions")
Pkg.add("CSV")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Compose
using Distributions
using Plots
using CSV
using DelimitedFiles

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
Decls.SetSpawnAndDestPts!(nw, [1, 3], [5])
sim = Decls.Simulation(nw, 60.0, maxIter = 200)

Decls.RunSim(sim)
CSV.write("output.csv", sim.simData)
writedlm("log.txt", Decls.simLog, "\n")

v = @linq sim.simData |> where(:agent .==  1) |> select(:lat, :lon)

compose(Visuals.DrawGraph(Decls.GetIntersectionCoords(nw)), Visuals.MarkPositions(v[:,1], v[:,2]))
