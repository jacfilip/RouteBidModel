module Visuals
# using Pkg
# Pkg.add("PyCall")
# Pkg.add("Conda")
# Pkg.add("OpenStreetMapX")
# Conda.runconda(`install folium -c conda-forge`)
using Conda
using OpenStreetMapX
using PyCall

function GetLLOfRoute(map::OpenStreetMapX.OSMData, mData::MapData,route::Array{Int64})
    myroute = []
    for nodeID in route
        latitude = map.nodes[mData.n[nodeID]].lat
        longitude = map.nodes[mData.n[nodeID]].lon
        push!(myroute,(latitude,longitude))
    end
    return myroute
end

function OVAGraph(map::OpenStreetMapX.OSMData, mData::MapData, a::Main.Decls.Agent)

    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    o_LL = GetLLOfRoute(map,mData,a.origRoute[1:end-1])
    t_LL = GetLLOfRoute(map,mData,a.travelledRoute[1:end])

    o_info = "Agent # $(a.id)\n<BR>"*
            "Length: $(length(a.origRoute)) nodes\n<br>" *
            "From: Node $(a.origRoute[1])\n<br>" *
            "To: Node $(a.origRoute[end-1])"
    t_info = "Agent $(a.id)\n<BR>"*
            "Length: $(length(a.travelledRoute)) nodes\n<br>" *
            "From: Node $(a.travelledRoute[1])\n<br>" *
            "To: Node $(a.travelledRoute[end])"

    flm.PolyLine(
            o_LL,
            popup=o_info,
            tooltip=o_info,
            color=matplotlib_colors.to_hex(cmap(1))
        ).add_to(m)
    flm.PolyLine(
            t_LL,
            popup=t_info,
            tooltip=t_info,
            color=matplotlib_colors.to_hex(cmap(2))
        ).add_to(m)

    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("OVAGraph.html")
    println("File Saved!")
end

function GraphAgents(map::OpenStreetMapX.OSMData, mData::MapData, agents::Array{Main.Decls.Agent})

    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    for n = 1:min(length(agents),10)
        LL = GetLLOfRoute(map,mData,agents[n].travelledRoute[1:end-1])
        info = "Agent # $(agents[n].id)\n<BR>"*
                "Length: $(length(agents[n].travelledRoute)) nodes\n<br>" *
                "From: Node $(agents[n].travelledRoute[1])\n<br>" *
                "To: Node $(agents[n].travelledRoute[end-1])\n<br>" *
                "Time Elapsed $(agents[n].arrivalTime)"
        flm.PolyLine(
                LL,
                popup=info,
                tooltip=info,
                color=matplotlib_colors.to_hex(cmap(n/min(length(agents),10)))
            ).add_to(m)
    end

    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("AgentsGraph.html")
    println("File Saved!")
end

end
