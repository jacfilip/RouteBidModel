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

#run(`$(PyCall.python) -m pip install folium`)

Random.seed!(0)

path = "maps"
file = "manhattan_filtered.osm"

cd("C:\\Users\\jacekf\\github\\RouteBidModel\\src")

nw = CreateNetworkFromFile(path * "//", file)
#SetSpawnAndDestPts!(nw, GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), GetNodesInRadius(nw,(-2000.,-2000.),2000.))

#Bridge1: 1044->1045, 532->1214 , 445->446
#SetSpawnAndDestPts!(nw, [345], [958])
inter = GetIntersectByNode(nw, 553)
SetSpawnAndDestPts!(nw,  GetNodesInRadius(nw, (inter.posX, inter.posY), 200), [847])
#disable lane no.1
GetRoadByNodes(nw, 1044, 1045).length = Inf
nw.graph.weights[1044, 1045] = Inf

#disable lane no.2
GetRoadByNodes(nw, 532, 1214).length = Inf
nw.graph.weights[532, 1214] = Inf

#disable lane no.3
GetRoadByNodes(nw, 445, 446).length = Inf
nw.graph.weights[445, 446] = Inf


Random.seed!(0)
sim = Simulation(nw, 30 * 60, maxAgents = 10000, dt = 5.0, initialAgents = 8000, auctions = false)
@time RunSim(sim)
SaveSim(sim, "sim_na_2k_t=90m")

#CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "manh_na_90m_1_agent_auctions.csv", DumpAuctionsInfo(sim))
#CSV.write(raw"C:\Users\jacekf\Dropbox\Canada2019\results\simulations" * "\\" * "manh_na_90m_1_agent_finished.csv", DumpFinishedAgents(sim))

#CSV.write(raw".\results\history.csv", sim.simData)
#CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
#CSV.write(raw".\results\auctions.csv", DumpAuctionsInfo(sim))
#CSV.write(raw".\results\finished.csv", DumpFinishedAgents(sim))
#writedlm(raw".\results\log.txt", simLog, "\n")


osmmap = OpenStreetMapX.parseOSM(path * "//" * file)
crop!(osmmap)
mData = get_map_data(path, file, only_intersections = true)

GraphAgents(osmmap, mData, sim.agentsFinished)
GraphNodesLocations(osmmap, mData)
GraphAllPaths(osmmap, mData)

for n in values(mData.n)
    println("Key: $(n), value: $(osmmap.nodes[n])")
end


end
