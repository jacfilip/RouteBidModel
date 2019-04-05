using Pkg

Pkg.add("LightGraphs")
Pkg.add("SimpleWeightedGraphs")
Pkg.add("OpenStreetMapX")

using Test
using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using RouteBidModel

#include("../src/decls.jl")

@test true
