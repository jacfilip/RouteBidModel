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
export GetNodesBetween
export LoadSim
export SaveSim
export LoadSim2
export SaveSim2
export GraphAgents

include("decls.jl")
include("osm_convert.jl")
include("Visuals.jl")

Random.seed!(0)

path = "maps//"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
#SetSpawnAndDestPts!(nw, GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), GetNodesInRadius(nw,(-2000.,-2000.),2000.))
SetSpawnAndDestPts!(nw, GetNodesBetween(nw,(-2000.,-2000.), 2000, 4000), GetNodesInRadius(nw,(-2000.,-2000.), 1000.))

Random.seed!(0)
sim = Simulation(nw, 90 * 60, maxAgents = 4000, dt = 10.0, initialAgents = 4000, auctions = true)
@time RunSim(sim)
SaveSim2(sim, raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\", "sim_stack_90m_4k_agents")
CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "sim_stack_90m_4k_agents_auctions.csv", DumpAuctionsInfo(sim))
CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "sim_stack_90m_4k_agents_finished.csv", DumpFinishedAgents(sim))

Random.seed!(0)
sim2 = Simulation(nw, 90 * 60, maxAgents = 4000, dt = 10.0, initialAgents = 4000, auctions = false)
@time RunSim(sim2)
SaveSim2(sim2, raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\", "sim_na_90m_4k_agents")
CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "sim_na_90m_4k_agents_auctions.csv", DumpAuctionsInfo(sim))
CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "sim_na_90m_4k_agents_finished.csv", DumpFinishedAgents(sim))

#stack = LoadSim("stack_scenario_75min\\sim_stack_4k_10dt_500ini_t=4500")


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

osmmap = OpenStreetMapX.parseOSM(path * "//" * file)
crop!(osmmap)
mData = get_map_data(path, file, only_intersections = true)

ag4503_na = GetAgentdByID_2(sim.agentsFinished, 4503)
ag4503_st = GetAgentdByID_2(stack.agentsFinished, 4503)

ag4712_na = GetAgentdByID_2(sim.agentsFinished, 4712)
ag4712_st = GetAgentdByID_2(stack.agentsFinished, 4712)

GraphAgents(osmmap, mData, sim.agentsFinished[1:2])

end
