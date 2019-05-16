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
using Serialization

include("../src/decls.jl")
include("../src/osm_convert.jl")
include("../src/Visuals.jl")

Random.seed!(0)

path = "maps//"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
Decls.SetSpawnAndDestPts!(nw, Decls.GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), Decls.GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Decls.Simulation(nw, 75 * 60 + 1, maxAgents = 4000, dt = 10.0, initialAgents = 500, auctions = true)
#sim2 = Decls.Simulation(nw, 2 * 60 , maxAgents = 4000, dt = 10.0, initialAgents = 500, auctions = true)

@time Decls.RunSim(sim, 40)

Decls.SaveSim(sim, "sim_stack_4k_10dt_500ini_t=1950")

sim = Decls.LoadSim("sim_stack_4k_10dt_500ini_t=1950")

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
CSV.write(raw".\results\coords.csv", Decls.GetIntersectionCoords(sim.network))
CSV.write(raw".\results\interInfo.csv", Decls.DumpIntersectionsInfo(nw, map, mData))
CSV.write(raw".\results\auctions.csv", Decls.DumpAuctionsInfo(sim))
write(raw".\results\auctions.txt", Decls.DumpAuctionsInfo(sim))
CSV.write(raw".\results\finished.csv", Decls.DumpFinishedAgents(sim))
writedlm(raw".\results\alternatecosts.csv", rcosts, "\n")

writedlm(raw".\results\log.txt", Decls.simLog, "\n")
writedlm(raw".\results\costs.csv", ttc, ",")

f = open(raw".\results\sim2.txt", "w")
serialize(f, sim)
close(f)

f = open(raw".\results\sim2.txt", "r")
sim = deserialize(f)
close(f)

 map = OpenStreetMapX.parseOSM(raw"maps\buffaloF.osm")
 crop!(map)
 mData = get_map_data("maps", "buffaloF.osm")

Visuals.GraphAgents(map, mData, sim.network.agents)

#6469
