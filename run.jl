using Revise
using Pkg
Pkg.activate(".")
using RouteBidModel
using Random
using OpenStreetMapX
using DelimitedFiles, CSV

Random.seed!(0);


path = joinpath("src","maps")
file = "manhattan_filtered.osm"

nw = create_network_from_file(path, file)
#set_spawn_dest!(nw,  [553], [847])

set_spawn_dest!(nw, vcat(get_nodes_in_radius(nw, (nw.intersections[718].posX, nw.intersections[718].posY), 100.0),
                                get_nodes_in_radius(nw, (nw.intersections[636].posX, nw.intersections[636].posY), 50.0),
                                get_nodes_in_radius(nw, (nw.intersections[54].posX, nw.intersections[54].posY), 100.0)),
                        vcat(get_nodes_in_radius(nw, (nw.intersections[958].posX, nw.intersections[958].posY), 100.0),
                                get_nodes_in_radius(nw, (nw.intersections[601].posX, nw.intersections[601].posY), 70.0),
                                get_nodes_in_radius(nw, (nw.intersections[663].posX, nw.intersections[663].posY), 50.0)))

CSV.write(raw".\results\scenario-1.csv", solve_scenario("scenario1", nw, 700))
CSV.write(raw".\results\scenario-4.csv", solve_scenario("scenario4", nw, 1000, 3.0, 24.0, 9623423))
CSV.write(raw".\results\scenario-5.csv", solve_scenario("scenario5",nw, 1000, 10.0, 24.0, 554618))
CSV.write(raw".\results\scenario-6.csv", solve_scenario("scenario6",nw, 1000, 3.0, 35.0, 827326))
CSV.write(raw".\results\scenario-8.csv", solve_scenario("scenario8",nw, 1000, 0.0, 24.0, 18226))

vot = 24 .+ randn(700) .* 3
vot[1:3] = [80, 90, 100]
vot[4:6] = [1, 5, 10]

vot = vot ./ 3600

CSV.write(raw".\results\scenario-3.csv", solve_scenario("scenario3",nw, 700, vot, 6456449))

osmmap = OpenStreetMapX.parseOSM(joinpath(path,file))
crop!(osmmap)
mData = get_map_data(path, file, only_intersections = true)
plot_agents(osmmap, mData, sim.agentsFinished)

# Random.seed!(0);
# const p = ModelParams()
# sim = Simulation(network=nw, p = p)
# for unused_lane in p.unused_lanes
#     sim.network.mapData.w[unused_lane...] = Inf
# end
#
# spawn_num_agents!(sim, p.initialAgents)
#
# opty = calculate_optimal_jump(sim)
#
# ns = calculate_nash(sim)
#
# runsim!(sim)
#savesim(sim, "sim_na_2k_t=90m")


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
