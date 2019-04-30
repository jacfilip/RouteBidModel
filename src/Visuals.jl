module Visuals

using IJulia
using Conda
using PyCall
using OpenStreetMapX

include("./decls.jl")

Conda.runconda(`install folium -c conda-forge`)

flm = pyimport("folium")
matplotlib_cm = pyimport("matplotlib.cm")
matplotlib_colors = pyimport("matplotlib.colors")

cmap = matplotlib_cm.get_cmap("prism")

map = OpenStreetMapX.parseOSM(raw".\maps\buffaloF.osm")
crop!(map)
mx = get_map_data("maps", "buffaloF.osm", only_intersections = true, use_cache = false)

node_ids = collect(keys(mx.nodes))
routes = Vector{Vector{Int}}()
visits = Dict{Int,Int}()
for i in 1:50
    a,b = [point_to_nodes(generate_point_in_bounds(mx), mx) for i in 1:2]
    route, route_time = OpenStreetMapX.shortest_route(mx,a,b)
    if route_time < Inf # when we select points neaer edges no route might be found
        push!(routes, route)
        for n in route
            visits[n] = get(visits, n,0)+1
        end
    end
end

SHOW_PATHS=20
m = flm.Map()
for k=1:SHOW_PATHS
    locs = [LLA(mx.nodes[n],mx.bounds) for n in routes[k]]
    info = "Sample route number $k\n<BR>"*
        "Length: $(length(routes[k])) nodes\n<br>" *
        "From: $(routes[k][1]) $(round.((locs[1].lat, locs[1].lon),digits=4))\n<br>" *
        "To: $(routes[k][end]) $(round.((locs[end].lat, locs[end].lon),digits=4))"
    flm.PolyLine(
        [(loc.lat, loc.lon) for loc in locs ],
        popup=info,
        tooltip=info,
        color=matplotlib_colors.to_hex(cmap(k/SHOW_PATHS))
    ).add_to(m)
end

MAP_BOUNDS = [(mx.bounds.min_y,mx.bounds.min_x),(mx.bounds.max_y,mx.bounds.max_x)]
flm.Rectangle(MAP_BOUNDS, color="black",weight=6).add_to(m)
m.fit_bounds(MAP_BOUNDS)

notebook()

end  #module
