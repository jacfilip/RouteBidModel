using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")
Pkg.add("OpenStreetMapXPlot")
Pkg.add("DataFrames")
Pkg.add("DataFramesMeta")
Pkg.add("Distributions")
Pkg.add("CSV")
Pkg.add("Compose")
Pkg.add("Random")
Pkg.add("IJulia")
Pkg.add("Conda")
Pkg.add("PyCall")

using Test
using OpenStreetMapX, OpenStreetMapXPlot
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Compose
using Distributions
using CSV
using DelimitedFiles
using SparseArrays
using Random
using Conda
using PyCall

include("../src/decls.jl")
include("../src/osm_convert.jl")
include("../src/Visuals.jl")

Random.seed!(0)

path = "/Users/arashdehghan/Desktop/RouteBidModel/maps/"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
Decls.SetSpawnAndDestPts!(nw, Decls.GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), Decls.GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Decls.Simulation(nw, 300.0, maxAgents = 100, dt = 5.0)

@time Decls.RunSim(sim)

map = OpenStreetMapX.parseOSM(path * file)
mData = get_map_data(path,file,only_intersections = true)

Visuals.OVAGraph(map,mData,sim.agentsFinished[1])
Visuals.GraphAgents(map,mData,sim.agentsFinished)


CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/history.csv", sim.simData)
CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/roadInfo.csv", sim.roadInfo)
CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/coords.csv", Decls.GetIntersectionCoords(sim.network))
writedlm("/Users/arashdehghan/Desktop/RouteBidModel/results/log.txt", Decls.simLog, "\n")
