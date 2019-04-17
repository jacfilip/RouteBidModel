using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")
Pkg.add("DataFrames")
Pkg.add("DataFramesMeta")
Pkg.add("Distributions")
Pkg.add("CSV")
Pkg.add("Compose")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Compose
using Distributions
using CSV
using DelimitedFiles

include("../src/decls.jl")
include("../src/Visuals.jl")
include("../src/osm_convert.jl")

nw = CreateNetworkFromFile(raw".\maps\buffaloF.osm")
Decls.SetSpawnAndDestPts!(nw, [1, 3], [5])
sim = Decls.Simulation(nw, 200.0)

@time Decls.RunSim(sim)

CSV.write(raw".\results\history.csv", sim.simData)
CSV.write(raw".\results\coords.csv", Decls.GetIntersectionCoords2(sim.network))
writedlm(raw".\results\log.txt", Decls.simLog, "\n")

v = @linq sim.simData |> where(:agent .==  1) |> select(:posX, :posY)

compose(Visuals.DrawGraph(Decls.GetIntersectionCoords(nw)), Visuals.MarkPositions(v[:,1], v[:,2]))

ii = dijkstra_shortest_paths(nw.graph,5).parents[1]
pth = Vector{Int}()

ii = 2
while ii != 5
    global ii = dijkstra_shortest_paths(nw.graph,5).parents[ii]
    push!(pth,ii)
    if length(pth) > 2000
        break
    end
end

sim.network.intersections[3037].posX, sim.network.intersections[3037].posY

poiss = Distributions.Poisson(5)
plot(pdf(poiss, 0:100))
rand(poiss)
