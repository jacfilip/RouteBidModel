using Pkg
Pkg.activate(".")
using RouteBidModel
using Random
using OpenStreetMapX
using Revise

Random.seed!(0);

path = joinpath("src","maps")
file = "manhattan_filtered.osm"

#set_spawn_dest!(nw, GetNodesOutsideRadius(nw,(-2000.,-2000.),4000.), get_nodes_in_radius(nw,(-2000.,-2000.),2000.))

#Bridge1: 1044->1045, 532->1214 , 445->446
#set_spawn_dest!(nw, [345], [958])
#inter = getintersect(nw, 553)
#disable lane no.1
# get_road_by_nodes(nw, 1044, 1045).length = Inf
# nw.graph.weights[1044, 1045] = Inf
#
# #disable lane no.2
# get_road_by_nodes(nw, 532, 1214).length = Inf
# nw.graph.weights[532, 1214] = Inf
#
# #disable lane no.3
# get_road_by_nodes(nw, 445, 446).length = Inf
# nw.graph.weights[445, 446] = Inf

nw = create_network_from_file(path, file)

set_spawn_dest!(nw,  [553], [847])

Random.seed!(0);
sim = Simulation(nw, 60 * 60, maxAgents = 1000, dt = 5.0, initialAgents = 500, auctions = false)
@time runsim(sim)
osmmap = OpenStreetMapX.parseOSM(joinpath(path,file))
crop!(osmmap)
mData = get_map_data(path, file, only_intersections = true)
plot_agents(osmmap, mData, sim.agentsFinished)
savesim(sim, "sim_na_2k_t=90m")

east_r = get_road_by_nodes(nw, 445, 446)
west_r = get_road_by_nodes(nw, 965, 112)
length(sim.agentsFinished)


#CSV.write(raw"C:\Users\jacek.filipowski\Dropbox\Canada2019\results\simulations" * "\\" * "manh_na_90m_1_agent_auctions.csv", DumpAuctionsInfo(sim))
#CSV.write(raw"C:\Users\jacekf\Dropbox\Canada2019\results\simulations" * "\\" * "manh_na_90m_1_agent_finished.csv", DumpFinishedAgents(sim))

#CSV.write(raw".\results\history.csv", sim.simData)
#CSV.write(raw".\results\roadInfo.csv", sim.roadInfo)
#CSV.write(raw".\results\auctions.csv", DumpAuctionsInfo(sim))
#CSV.write(raw".\results\finished.csv", DumpFinishedAgents(sim))
#writedlm(raw".\results\log.txt", simLog, "\n")

plot_agents(osmmap, mData, sim.agentsFinished)
plot_nodes_locations(osmmap, mData)
plot_all_paths(osmmap, mData)

for n in values(mData.n)
    println("Key: $(n), value: $(osmmap.nodes[n])")
end
