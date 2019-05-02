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

include("../src/decls.jl")
include("../src/Visuals.jl")
include("../src/osm_convert.jl")

Random.seed!(0)

nw = CreateNetworkFromFile("maps", "buffaloF.osm")
Decls.SetSpawnAndDestPts!(nw, Decls.GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), Decls.GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Decls.Simulation(nw, 105.0, maxAgents = 1000, dt = 5.0)

@time Decls.RunSim(sim)

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
CSV.write(raw".\results\coords.csv", Decls.GetIntersectionCoords(sim.network))
writedlm(raw".\results\log.txt", Decls.simLog, "\n")

#test czy alter route cost > best route cost

r = Decls.GetRoadByNodes(nw, 3743, 2700)
Decls.GetMR(nw.agents[80], r, sim.timeElapsed)

Decls.GetRoadByNodes(nw, 2768, 2769)

ag1 = nw.agents[1]
