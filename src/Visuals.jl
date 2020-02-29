function getLL_of_route(mData::MapData,route::AbstractArray{Int})
    myroute = []
    jitter = 4e-5
    for nodeID in route
        lla = LLA(mData.nodes[nodeID],mData.bounds)
        #push!(myroute,(lla.lat + jitter * randn(),lla.lon + jitter * randn()))
        push!(myroute,(lla.lat,lla.lon))
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
    m = flm.Map(tiles="Stamen Toner")

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

function plot_agents(s::Simulation, title::String; agentIds=1:min(1000,s.network.agentIDmax))
    println("plot_agents")
    mData = s.network.mapData
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map(tiles="Stamen Toner")

    segment_cs = Dict{Tuple{Int,Int},Int}()
    for n in agentIds
        a = s.network.agents[n]
        myroute = a.routes[a.travelledRoute]
        segments =  [(myroute[i],myroute[i+1]) for i in 1:length(myroute)-1]
        for segment in segments
            nn = get!(segment_cs,segment,0)
            segment_cs[segment] = nn+1
        end
    end
    max_count = maximum(values(segment_cs))
    min_count = minimum(values(segment_cs))
    cc = max_count - min_count + 1
    cols = reshape( range(colorant"blue", stop=colorant"red",length=cc), 1, cc);

    for segment in keys(segment_cs)
        a_count = segment_cs[segment]
        colix = a_count + 1 - min_count
        LL = getLL_of_route(s.network.mapData, [segment...])
        info =  "From: Node $(segment[1])\n<br>" *
                "To: Node $(segment[2])\n<br>" *
                "Load: $(a_count)"
        flm.PolyLine(
                LL,
                popup=info,
                tooltip=info,
                weight=round(Int,log(7*colix))+1,
                color="#$(hex(cols[colix]))"
            ).add_to(m)
    end

    MAP_BOUNDS = [(mData.bounds.min_y,mData.bounds.min_x),(mData.bounds.max_y,mData.bounds.max_x)]
    flm.Rectangle(MAP_BOUNDS, color="green",weight=6).add_to(m)
    m.fit_bounds(MAP_BOUNDS)
    dest = joinpath("results", title * ".html")
    m.save(dest)
    println("File saved at: " * dest)
end


function plot_nodes_locations(map::OpenStreetMapX.OSMData, mData::MapData)
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map(tiles="Stamen Toner")

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

function plot_all_paths(map::OpenStreetMapX.OSMData, mData::MapData)
    flm = pyimport("folium")
    matplotlib_cm = pyimport("matplotlib.cm")
    matplotlib_colors = pyimport("matplotlib.colors")
    cmap = matplotlib_cm.get_cmap("prism")
    m = flm.Map(tiles="Stamen Toner")

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
