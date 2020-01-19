using Revise
using Pkg
Pkg.activate(".")
using RouteBidModel
using Random
using OpenStreetMapX

Random.seed!(0);


path = joinpath("src","maps")
file = "manhattan_filtered.osm"

nw = create_network_from_file(path, file)

set_spawn_dest!(nw,  [553], [847])

Random.seed!(0);
const p = ModelParams()
sim = Simulation(network=nw, p = p)
for unused_lane in p.unused_lanes
    sim.network.mapData.w[unused_lane...] = Inf
end

spawn_num_agents!(sim, p.initialAgents)



runsim!(sim)
#savesim(sim, "sim_na_2k_t=90m")

#osmmap = OpenStreetMapX.parseOSM(joinpath(path,file))
#crop!(osmmap)
#mData = get_map_data(path, file, only_intersections = true)
plot_agents(sim,agentIds=1:10)

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
