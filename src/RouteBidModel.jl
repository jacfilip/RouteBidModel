module RouteBidModel

using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")
Pkg.add("DataFrames")
Pkg.add("DataFramesMeta")
Pkg.add("Distributions")
Pkg.add("CSV")
Pkg.add("Compose")
Pkg.add("Colors")
Pkg.add("Plots")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using DataFrames, DataFramesMeta
using Distributions
using CSV
using Compose

include("./decls.jl")

#include("./run.jl")

end  # module RouteBidModel
