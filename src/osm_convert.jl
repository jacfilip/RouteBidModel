module graph

include("./decls.jl")

using OpenStreetMapX
using OpenStreetMapXPlot
using LightGraphs, SimpleWeightedGraphs
using GraphPlot
using Plots
using Compose, Colors


#cd(raw"C:\Users\jacek.filipowski\github")
path = raw"C:\Users\jacek.filipowski\github\route-bid-model\maps"
mapName = "buffaloF.osm"

map = OpenStreetMapX.parseOSM(raw"C:\Users\jacek.filipowski\github\route-bid-model\maps\buffaloF.osm")
crop!(map)

mData = get_map_data(path, mapName, only_intersections = true)
mapPlot = OpenStreetMapXPlot.plotmap(mData,width=1000,height=800)

nw = Decls.Network(mData)

function ConvToNetwork(map::MapData)::Decls.Network
    g =  SimpleWeightedDiGraph()
    add_vertices!(g, length(map.nodes))

    n = Decls.Network(g)
    return n
end

end  # module
