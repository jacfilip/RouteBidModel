include("./decls.jl")

using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using GraphPlot
using Plots
using Compose, Colors

path = raw"C:\Users\jacek.filipowski\github\route-bid-model\maps"
mapName = "buffaloF.osm"

function CreateNetworkFromFile(filePath::String)::Decls.Network
    map = OpenStreetMapX.parseOSM(filePath)
    crop!(map)
    mData = get_map_data(path, mapName, only_intersections = true)
    nw = Decls.Network(mData)
    mapPlot = OpenStreetMapXPlot.plotmap(mData,width=1000,height=800)
    return nw
end

# map = OpenStreetMapX.parseOSM(raw"C:\Users\jacek.filipowski\github\route-bid-model\maps\buffaloF.osm")
# crop!(map)
# mData = get_map_data(path, mapName, only_intersections = true)
