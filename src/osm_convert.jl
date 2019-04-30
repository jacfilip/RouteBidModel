include("./decls.jl")

using OpenStreetMapX
using LightGraphs, SimpleWeightedGraphs
using GraphPlot
using Plots
using Compose, Colors

#path = raw"C:\Users\jacek.filipowski\github\route-bid-model\maps"
#mapName = "buffaloF.osm"

function CreateNetworkFromFile(path::String, mapName::String)::Decls.Network
    map = OpenStreetMapX.parseOSM(path * "\\" * mapName)
    crop!(map)

    mData = get_map_data(path, mapName, only_intersections = true, use_cache = false)
    nw = Decls.Network(mData)
    #mapPlot = OpenStreetMapXPlot.plotmap(mData,width=1000,height=800)
    return nw
end

#map = OpenStreetMapX.parseOSM(raw"C:\Users\jacek.filipowski\github\route-bid-model\maps\buffaloF.osm")
#crop!(map)
#mData = get_map_data(path, mapName, only_intersections = true)

map = OpenStreetMapX.parseOSM("maps" * "\\" * "buffaloF.osm")
mData = get_map_data("maps", "buffaloF.osm", only_intersections = true)
