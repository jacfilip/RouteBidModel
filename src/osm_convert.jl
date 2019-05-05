include("./decls.jl")

using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using GraphPlot
using Plots
using Compose, Colors

function CreateNetworkFromFile(path::String, mapName::String)::Decls.Network
    map = OpenStreetMapX.parseOSM(path * mapName)
    crop!(map)

    mData = get_map_data(path, mapName, only_intersections = true)
    nw = Decls.Network(mData)
    return nw
end


# map = OpenStreetMapX.parseOSM("maps" * "\\" * "buffaloF.osm")
# crop!(map)
# mData = get_map_data("maps", "buffaloF.osm", only_intersections = true)

