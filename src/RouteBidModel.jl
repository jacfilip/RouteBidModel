module RouteBidModel

using Test
using Random
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Distributions
using CSV
using Compose, Colors
using Conda
using PyCall
using GraphPlot
using Plots
using SparseArrays
using Serialization
using PaddedViews
using RecursiveArrayTools

export Network
export Simulation
export Agent
export Road
export Intersection
export SetLL
export MakeAction!
export RunSim
export SetSpawnAndDestPts!
export SpawnAgentAtRandom
export GetNodesOutsideRadius
export GetNodesInRadius
export LoadSim
export SaveSim

include("decls.jl")
include("osm_convert.jl")
include("Visuals.jl")

Random.seed!(0)

path = "/Users/arashdehghan/Desktop/RouteBidModel/maps/"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
SetSpawnAndDestPts!(nw, GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Simulation(nw, 75 * 60, maxAgents = 4000, dt = 10.0, initialAgents = 500, auctions = true)
# sim = Simulation(nw, 1500, maxAgents = 4000, dt = 10.0, initialAgents = 500, auctions = true)

@time RunSim(sim)

SaveSim(sim, "sim_hungarian_4k_10dt_500ini")
# sim = LoadSim("sim_hungarian_4k_10dt_500ini")

# map = OpenStreetMapX.parseOSM("/Users/arashdehghan/Desktop/RouteBidModel/maps/buffaloF.osm")
# crop!(map)
# mData = get_map_data("/Users/arashdehghan/Desktop/RouteBidModel/maps/", "buffaloF.osm")

# OVAGraph(map, mData, a)
# GraphAgents(map,mData,sim.agentsFinished)


# CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/history.csv", sim.simData)
# CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/roadInfo.csv", sim.roadInfo)
### CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/coords.csv", GetIntersectionCoords(sim.network))
### CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/interInfo.csv", DumpIntersectionsInfo(nw, map, mData))
# CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/auctions.csv", DumpAuctionsInfo(sim))
# CSV.write("/Users/arashdehghan/Desktop/RouteBidModel/results/finished.csv", DumpFinishedAgents(sim))
### writedlm("/Users/arashdehghan/Desktop/RouteBidModel/results/log.txt", simLog, "\n")

end

