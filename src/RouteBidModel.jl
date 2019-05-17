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

path = "maps//"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
SetSpawnAndDestPts!(nw, GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Simulation(nw, 75 * 60, maxAgents = 4000, dt = 10.0, initialAgents = 500, auctions = true)

sim = LoadSim("sim_stack_4k_10dt_500ini_t=1910_before_auction")

@time RunSim(sim, 20)

SaveSim(sim, "sim_stack_4k_10dt_500ini_t=1910_before_auction")

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
#CSV.write(raw".\results\coords.csv", GetIntersectionCoords(sim.network))
#CSV.write(raw".\results\interInfo.csv", Decls.DumpIntersectionsInfo(sim.network, map, mData))
CSV.write(raw".\results\auctions.csv", DumpAuctionsInfo(sim))
CSV.write(raw".\results\finished.csv", DumpFinishedAgents(sim))
writedlm(raw".\results\log.txt", simLog, "\n")

# using DataFrames, Query
# q1 = @from i in sim.roadInfo begin
#      @where i.k >= 10 && (i.k / i.k_max) >= 0.8
#      @select {i.iter, i.bNode, i.fNode, i.k, i.k_max}
#      @collect DataFrame
#  end
# CSV.write(raw".\results\roadInfo.csv", q1)

end
