module RouteBidModel

Pkg.add("OpenStreetMapX")
Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs


include("./decls.jl")

#include("./run.jl")

end  # module RouteBidModel
