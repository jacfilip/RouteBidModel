function GetLLOfRoute(map::OpenStreetMapX.OSMData, mData::MapData,route::Array{Int64})
    myroute = []
    for nodeID in route
        latitude = map.nodes[mData.n[nodeID]].lat
        longitude = map.nodes[mData.n[nodeID]].lon
        push!(myroute,(latitude,longitude))
    end
    return myroute
end

function GetLLOfPoint(map::OpenStreetMapX.OSMData, mData::MapData,route::Int64)
    latitude = map.nodes[mData.n[route]].lat
    longitude = map.nodes[mData.n[route]].lon
    mypoint = (latitude,longitude)
    return mypoint
end

function GetLLOfPoint2(map::OpenStreetMapX.OSMData, mData::MapData,n::Int64)
    latitude = map.nodes[n].lat
    longitude = map.nodes[n].lon
    mypoint = (latitude,longitude)
end

function GrabBannedRoads(roads::Array{Road})
    r_list = []
    for r in roads
        push!(r_list,[r.bNode,r.fNode])
    end

    return r_list
end

function OVAGraph(map::OpenStreetMapX.OSMData, mData::MapData, a::Agent)

    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    o_LL = GetLLOfRoute(map,mData,a.origRoute[1:end])
    t_LL = GetLLOfRoute(map,mData,a.travelledRoute[1:end])

    o_info = "Original Route\n<BR>"*
            "Agent # $(a.id)\n<BR>"*
            "Length: $(length(a.origRoute)) nodes\n<br>" *
            "From: Node $(a.origRoute[1])\n<br>" *
            "To: Node $(a.origRoute[end])"
    t_info = "Route Travelled\n<BR>"*
            "Agent $(a.id)\n<BR>"*
            "Length: $(length(a.travelledRoute)) nodes\n<br>" *
            "From: Node $(a.travelledRoute[1])\n<br>" *
            "To: Node $(a.travelledRoute[end])"

    flm.PolyLine(
            o_LL,
            popup=o_info,
            tooltip=o_info,
            color="blue"
        ).add_to(m)
    flm.PolyLine(
            t_LL,
            popup=t_info,
            tooltip=t_info,
            color="red"
        ).add_to(m)

    if isempty(a.bannedRoads) == false
        br = GrabBannedRoads(a.bannedRoads)
        for banned_road in br
            b_LL = GetLLOfRoute(map,mData,banned_road)

            b_info = "Banned Road\n<BR>"

            flm.PolyLine(
                    b_LL,
                    popup=b_info,
                    tooltip=b_info,
                    color="purple"
                ).add_to(m)

        end
    end

    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("OVAGraph.html")
    println("File Saved!")
end

function GraphAgents(map::OpenStreetMapX.OSMData, mData::MapData, agents::Array{Agent})

    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    for n = 1:min(length(agents),10)
        LL = GetLLOfRoute(map,mData,agents[n].travelledRoute[1:end])
        info = "Agent # $(agents[n].id)\n<BR>"*
                "Length: $(length(agents[n].travelledRoute)) nodes\n<br>" *
                "From: Node $(agents[n].travelledRoute[1])\n<br>" *
                "To: Node $(agents[n].travelledRoute[end])\n<br>" *
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


function GraphAuctionLocations(map::OpenStreetMapX.OSMData, mData::MapData, aucs::Array{Auction})
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    for auction in aucs
        location = GetLLOfPoint(map,mData,auction.road.bNode)
        info = "Node $(auction.road.bNode)\n<br>"*
               "Auction ID $(auction.auctionID)\n<br>"*
               "Number of Agents Bought Off: $(length(auction.sBids))"
        flm.Circle(
         location,
         popup=info,
         tooltip=info,
         radius=50,
         color="crimson",
         weight=0.1,
         fill=true,
         fill_color="crimson"
      ).add_to(m)
    end
    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("AuctionsGraph.html")
    println("File Saved!")

end

function GraphNodesLocations(map::OpenStreetMapX.OSMData, mData::MapData)
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    for n in keys(mData.n)
        if !haskey(map.nodes, mData.n[n])
            println("Node $n not found")
            continue
        else
            println("Node $n marked on the map.")
        end
        location = GetLLOfPoint(map,mData,n)

        info = "Node $n\n<br>"
        flm.Circle(
         location,
         popup=info,
         tooltip=info,
         radius=10,
         color="crimson",
         weight=4,
         fill=true,
         fill_color="crimson"
      ).add_to(m)
    end
    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("NodesGraph.html")
    println("File Saved!")

end

function GraphAllPaths(map::OpenStreetMapX.OSMData, mData::MapData)
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map()

    for e in mData.e
        if !haskey(map.nodes, e[1]) || !haskey(map.nodes, e[2])
            continue
        end
        p1 = GetLLOfPoint2(map,mData,e[1])
        p2 = GetLLOfPoint2(map,mData,e[2])

        flm.PolyLine([p1, p2], color="red", weight=2.5, opacity=1).add_to(m)
    end
    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    m.save("RoadsGraph.html")
    println("File Saved!")
end
