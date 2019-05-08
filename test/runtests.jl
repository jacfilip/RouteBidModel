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

path = "maps//"
file = "buffaloF.osm"

nw = CreateNetworkFromFile(path, file)
Decls.SetSpawnAndDestPts!(nw, Decls.GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), Decls.GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Decls.Simulation(nw, 45.0, maxAgents = 1000, dt = 10.0, initialAgents = 500)

@time Decls.RunSim(sim)

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
CSV.write(raw".\results\coords.csv", Decls.GetIntersectionCoords(sim.network))
CSV.write(raw".\results\interInfo.csv", Decls.DumpIntersectionsInfo(nw, map, mData))
writedlm(raw".\results\log.txt", Decls.simLog, "\n")

map = OpenStreetMapX.parseOSM(raw"maps\buffaloF.osm")
crop!(map)
mData = get_map_data("maps", "buffaloF.osm")

Visuals.GraphAgents(map, mData, nw.agents)

#test czy alter route cost > best route cost
pth = dijkstra_shortest_paths(nw.graph,4228)
