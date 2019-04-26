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

using Test
using OpenStreetMapX, OpenStreetMapXPlot
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Compose
using Distributions
using CSV
using DelimitedFiles
using SparseArrays

include("../src/decls.jl")
include("../src/Visuals.jl")
include("../src/osm_convert.jl")

nw = CreateNetworkFromFile(raw".\maps\buffaloF.osm")
Decls.SetSpawnAndDestPts!(nw, Decls.GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), Decls.GetNodesInRadius(nw,(-2000.,-2000.),2000.))

sim = Decls.Simulation(nw, 20.0, maxAgents = 1000, dt_min = 2.5)

@time Decls.RunSim(sim)

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\coords.csv", Decls.GetIntersectionCoords2(sim.network))
writedlm(raw".\results\log.txt", Decls.simLog, "\n")

pth = Vector{Int}()
from, to = 1, 1004

k = from
while k != to
    global k = dijkstra_shortest_paths(nw.graph,to).parents[k]
    push!(pth,k)
    if length(pth) > 2000
        break
    end
end

yen = LightGraphs.yen_k_shortest_paths(nw.agents[1].reducedGraph,from,to,nw.graph.weights,2)

g = SimpleWeightedDiGraph(3)

add_edge!(g,3,1,5.0)
add_edge!(g,2,1,6.0)
add_edge!(g,3,2,7.0)
add_edge!(g,2,3,8.0)
add_edge!(g,1,3,9.0)

s = LightGraphs.SimpleDiGraph(g.weights.n,)
add_edge!(s,3,1)
add_edge!(s,2,1)
add_edge!(s,3,2)
add_edge!(s,1,2)
add_edge!(s,1,3)
add_edge!(s,2,3)
s = g
dij = dijkstra_shortest_paths(g,3)
yen = yen_k_shortest_paths2(nw.graph,2,3,mt,2)

mt = g.weights
mt[2,1] = 100.0

SimpleDiGraph(nw.graph.weights.n,)
LightGraphs.weights(s)

kk = [k for k in g.weights]

GraphPlot.gplot(nw.agents[1].reducedGraph)
