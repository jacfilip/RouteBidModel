using Pkg

Pkg.add("LightGraphs")
Pkg.add("OpenStreetMapX")

using Test
using OpenStreetMapX
using LightGraphs
using RouteBidModel

include("./decls.jl")

@test false
