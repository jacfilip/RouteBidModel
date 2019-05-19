### PARAMS

using Random
Random.seed!(0);

bigNum = 1000000

headway = 0.5
avgCarLen = 5.

auctionTimeInterval = 20.0 #60.0
auctionMinCongestion = 0.5 #0.8
auctionMinParticipants = 5 #10
auctionMinAgentsOnRoad = 5 #10
auctionsAtOnceMax = 5

muteRegistering = false
simLog = Vector{String}()

saveEach = 90  #in iterations

path = "maps"
file = "buffaloF.osm"

####

function AddRegistry(msg::String, prompt::Bool = false)
    if muteRegistering return end

    push!(simLog, msg)
    if prompt
        println(msg)
    end
end

"--------------------------------------------------------------"

function GrabZeroLocations(matrix::Array{Float64})::Array{Bool}
        zero_locations = falses(size(matrix))
        for ind = 1:length(matrix)
                zero_locations[ind] = (matrix[ind] == 0)
        end
        return zero_locations
end

function ConvertMatrix(matrix::Array{Float64}, size::Int64)
        # subtract profit matrix from a matrix made of the max value of the profit matrix
        offset_matrix = ones(Float64,(size,size)) * maximum(matrix)
        cost_matrix = offset_matrix - matrix
        return cost_matrix
end

mutable struct Hungarian
        original_matrix::Array{Float64}
        cost_matrix::Array{Float64}
        max_rows::Int64
        max_columns::Int64
        matrix_size::Int64
        matrix::Array{Float64}
        results::Array{Tuple{Int64,Int64}}
        totalPotential::Float64

        Hungarian(m::Array{Float64}) = (
        h = new();
        h.original_matrix = m;
        h.max_rows = size(m,1);
        h.max_columns = size(m,2);
        h.matrix_size = max(h.max_rows,h.max_columns);
        h.matrix = Array(PaddedView(0, m, (Base.OneTo(h.matrix_size), Base.OneTo(h.matrix_size))));
        h.cost_matrix = ConvertMatrix(h.matrix, h.matrix_size);
        h.results = [];
        h.totalPotential = 0;
        return h;
        )::Hungarian

end

mutable struct CoverZeros
        zero_locations::Array{Bool}
        mshape::Tuple{Int64,Int64}
        choices::Array{Bool}
        marked_rows::Vector{Int64}
        marked_columns::Vector{Int64}
        covered_rows::Vector{Int64}
        covered_columns::Vector{Int64}
        matrix::Array{Float64}

        CoverZeros(m::Array{Float64}) = (
        c = new();
        c.matrix = m;
        c.zero_locations = GrabZeroLocations(m);
        c.mshape = size(m);
        c.choices = falses(c.mshape);
        c.marked_rows = Vector{Int64}();
        c.marked_columns = Vector{Int64}();
        c.covered_rows = Vector{Int64}();
        c.covered_columns = Vector{Int64}();
        return c;
        )::CoverZeros

end

"--------------------------------------------------------------"

mutable struct Road
    length::Real
    agents::Vector{Int}
    bNode::Int
    fNode::Int
    vMax::Real
    vMin::Real
    curVelocity::Real
    capacity::Int
    lanes::Int
    ttime::Real
    MTS::Real   #Marginal Time Saving: travel time saved by removing one commuter

    Road(in::Int, out::Int, len::Float64, vel::Float64, lanes::Int = 1) = (
    r = new();
    r.bNode = in;
    r.fNode = out;
    r.lanes = lanes;
    r.length = len;
    r.vMax = vel;
    r.vMin = 1. /3.6;
    r.agents = Vector{Int}();
    r.capacity = ceil(r.length / (avgCarLen + headway)) * r.lanes;
    r.curVelocity = 0.0;
    r.ttime = r.length / r.vMax;
    r.MTS = 0.;
    return r;
    )::Road
end

mutable struct Intersection
    nodeID::Int
    posX::Float64
    posY::Float64
    inRoads::Vector{Road}
    outRoads::Vector{Road}
    spawnPoint::Bool
    destPoint::Bool
    lat::Real
    lon::Real

    Intersection(id::Int, posX = 0.0, posY = 0.0, spawnPoint = false, destPoint = false) =(
        inter = new();
        inter.nodeID = id;
        inter.posX = posX;
        inter.posY = posY;
        inter.lat = 0.0;
        inter.lon = 0.0;
        inter.spawnPoint = spawnPoint;
        inter.destPoint = destPoint;
        inter.inRoads = Vector{Road}();
        inter.outRoads = Vector{Road}();
        return inter;
    )::Intersection
end

mutable struct Agent
    id::Int
    atRoad::Union{Road,Nothing}
    roadPosition::Real
    atNode::Union{Intersection,Nothing}
    destNode::Intersection
    bestRoute::Vector{Int}
    alterRoute::Vector{Int}
    origRoute::Vector{Int}
    travelledRoute::Vector{Int}
    timeEstim::Real
    deployTime::Real
    requiredArrivalTime::Real
    arrivalTime::Real
    spareTime::Real
    reducedGraph::SimpleWeightedDiGraph
    #roadCosts::SparseArrays.SparseMatrixCSC{Float64,Int64}
    #roadBids::Vector{Tuple{Road, Real}}
    VoT_base::Real          # Base value of time $/min
    VoT_dev::Real           # value of time deviation regards of time remained -
    CoF::Real               # fuel cost $/m
    bestRouteCost::Real
    alterRouteCost::Real
    totalTravelCost::Real
    carLength::Real
    vMax::Real      #maximal car velocity in km/h
    moneySpent::Dict{Int,Real}
    moneyReceived::Dict{Int,Real}
    bannedRoads::Vector{Road}

    isSmart::Bool

    Agent(id::Int, start::Intersection, dest::Intersection, graph::SimpleWeightedDiGraph, deployTime::Real, arrivTime::Real) = (
        a = new();
        a.atNode = start;
        a.destNode = dest;
        a.reducedGraph = deepcopy(graph);

        a.id = id;
        a.roadPosition = 0.0;
        a.VoT_base = maximum([rand(Distributions.Normal(24.52/60.0/60.0, 3.0/60.0/60.0)), 0.0]);
        a.VoT_dev =  1.0 / (rand() * 8.0 + 2.0);
        a.CoF = 0.15e-3;         #ToDo: Check and draw value
        a.carLength = 3.0;      #m
        a.vMax = 120.0 / 3.6;         #m/s
        a.bestRoute = Vector{Int}();
        a.alterRoute = Vector{Int}();
        a.origRoute = Vector{Int}();
        a.travelledRoute = Vector{Int}();
        a.atRoad = nothing;
        a.deployTime = deployTime;
        a.timeEstim = 0.;
        a.requiredArrivalTime = arrivTime;
        a.arrivalTime = 0.;

        a.bestRouteCost = 0.;
        a.alterRouteCost = 0.;
        a.spareTime = 0.;
        a.totalTravelCost = 0.;
        a.isSmart = true;
        a.moneySpent = Dict{Int,Real}();
        a.moneyReceived = Dict{Int,Real}();
        a.bannedRoads = Vector{Road}();
        return  a;
    )::Agent
end

mutable struct Network
    roads::Vector{Road}
    intersections::Vector{Intersection}
    spawns::Vector{Int}
    dests::Vector{Int}
    numRoads::Int
    agents::Vector{Agent}
    graph::SimpleWeightedDiGraph
    osmData::OpenStreetMapX.OSMData
    mapData::MapData
    agentCntr::Int
    agentIDmax::Int
    Network(g::SimpleWeightedDiGraph, coords::Vector{Tuple{Float64,Float64,Float64}}, mdata::MapData) = (
            n = new();
            n.graph = deepcopy(g);
            n.numRoads = 0;
            n.spawns = Vector{Intersection}();
            n.dests = Vector{Intersection}();
            n.agents = Vector{Agent}();
            n.mapData = mdata;
            n.agentCntr = 0;
            n.agentIDmax = 0;
            InitNetwork!(n, coords);
            return n)::Network
    Network(g::SimpleWeightedDiGraph, coords::Vector{ENU}, m::OpenStreetMapX.MapData) = (
        return Network(g,[(i.east, i.north, i.up) for i in coords], m))::Network
    Network(map::MapData) = (return  ConvertToNetwork(map))::Network
end

mutable struct Auction
    auctionID::Int
    participants::Vector{Agent}
    road::Road
    MRs::Vector{Dict{Int, Real}}
    sBids::Vector{Dict{Int, Real}}
    bBids::Vector{Dict{Int,Real}}
    clearingPrice::Vector{Real}
    winners::Vector{Agent}
    rounds::Int
    time::Real
    payments::SparseMatrixCSC{Real,Int}

    Auction(id::Int, participants::Vector{Agent}, road::Road, time::Real) = (
        au = new();
        au.participants = participants;
        au.road = road;
        au.auctionID = id;
        au.rounds = 0;
        au.time = time;
        au.MRs = Vector{Dict{Int, Real}}();
        au.sBids = Vector{Dict{Int, Real}}();
        au.bBids = Vector{Dict{Int, Real}}();
        au.clearingPrice = Vector{Real}();
        au.winners = Vector{Agent}();
        au.payments = spzeros(maximum(getfield.(participants, :id)), maximum(getfield.(participants, :id)));
    return au)::Auction
end

mutable struct Simulation
    network::Network
    timeMax::Real
    timeStep::Real
    timeStepVar::Vector{Real}
    dt_min::Real
    iter::Int

    simData::DataFrame
    roadInfo::DataFrame
    simLog::Vector{String}

    maxAgents::Int
    maxIter::Int
    initialAgents::Int
    agentsFinished::Vector{Agent}

    auctionCntr::Int
    enableAuctions::Bool
    auctions::Vector{Auction}
    lastAuctionTime::Real

    timeElapsed::Real
    isRunning::Bool
    Simulation(n::Network, tmax::Real; dt::Real = 0, dt_min = 1.0, maxAgents::Int = bigNum, maxIter::Int = bigNum, run::Bool = true, initialAgents::Int = 200, auctions::Bool = true) = (
        s = Simulation();
        s.network = n;
        s.agentsFinished = Vector{Agent}();
        s.iter = 0;
        s.timeMax = tmax;
        s.timeStep = dt;
        s.dt_min = dt_min;
        s.timeStepVar = Vector{Real}();
        s.timeElapsed = 0;
        s.maxAgents = maxAgents;
        s.initialAgents = initialAgents;
        s.isRunning = run;
        s.maxIter = maxIter;
        s.lastAuctionTime = 0.0;
        s.auctionCntr = 0;
        s.enableAuctions = auctions;
        s.auctions = Vector{Auction}();
        s.simData = DataFrame(  iter = Int[],
                               t = Real[],
                               agent = Int[],
                               node1 = Int[],
                               node2 = Int[],
                               roadPos = Real[],
                               posX = Real[],
                               posY = Real[],
                               v = Real[],
                               t_est = Real[],
                               vot = Real[],
                               spare_t = Real[],
                               arrival = Real[]
                               );
        s.roadInfo = DataFrame( iter = Int[],
                                bNode = Int[],
                                fNode = Int[],
                                len = Real[],
                                k = Int[],
                                k_max = Int[],
                                v = Real[]
                                );
        return s;)::Simulation
    Simulation() = new()
end

# function GetTimeStep(s::Simulation)::Real
#     if s.timeStep != 0
#         return s.timeStep
#     elseif length(s.timeStepVar) > 0
#         return s.timeStepVar[end]
#     else
#         return 0
#     end
#
# end
#
# function SetTimeStep!(s::Simulation)::Real
#     if s.timeStep != 0
#         push!(s.timeStepVar, s.timeStep)
#     else
#         t_min = Inf
#          for r in s.network.roads
#              if !isempty(r.agents)
#                 if (t = (r.length - GetAgentByID(s.network, r.agents[1]).roadPosition) / r.curVelocity) < t_min
#                     t_min = t
#                 end
#              end
#          end
#
#          t_min = t_min == Inf ? 0.0 : t_min
#
#          if t_min < 0
#             throw(ErrorException("Error, simulation step is negative!"))
#         end
#         push!(s.timeStepVar, maximum([t_min, s.dt_min]))
#     end
#     #AddRegistry("Time step: $(s.timeStepVar[s.iter]) at iter $(s.iter)")
#     return s.timeStepVar[end]
# end

function InitNetwork!(n::Network, coords::Vector{Tuple{Float64,Float64,Float64}})
    n.intersections = Vector{Intersection}(undef,n.graph.weights.m)
    for i in 1:n.graph.weights.m
        n.intersections[i] = Intersection(i, coords[i][1], coords[i][2])
        n.intersections[i].lat = LLA(ENU(n.intersections[i].posX, n.intersections[i].posY, 0), n.mapData.bounds).lat
        n.intersections[i].lon = LLA(ENU(n.intersections[i].posX, n.intersections[i].posY, 0), n.mapData.bounds).lon
    end

    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                n.numRoads += 1
                AddRegistry("$(n.numRoads): $(i) -> $(j) = $(n.graph.weights[i,j])")
            end
        end
    end
    n.roads = Vector{Road}(undef, n.numRoads)

    r = 0
    vels = OpenStreetMapX.get_velocities(n.mapData)
    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                r += 1
                n.roads[r] = Road(i, j, n.graph.weights[i, j], (i <= vels.m && j <= vels.n) && vels[i,j] > 0 ? vels[i, j] : 40 / 3.6)
                push!(n.intersections[i].outRoads, n.roads[r])
                push!(n.intersections[j].inRoads, n.roads[r])
            end
        end
    end
    AddRegistry("Network has been successfully initialized.", true)
end

function ConvertToNetwork(m::MapData)::Network
    g =  SimpleWeightedDiGraph()
    add_vertices!(g, length(m.n))
    for edge in m.e
    #    dist = DistanceENU(m.nodes[edge[1]], m.nodes[edge[2]])
        dist = m.w[m.v[edge[1]], m.v[edge[2]]]
        add_edge!(g, m.v[edge[1]], m.v[edge[2]], dist)
        add_edge!(g, m.v[edge[2]], m.v[edge[1]], dist)
    end

    return Network(g, [m.nodes[m.n[i]] for i in 1:length(m.n)], m)
end

function SetSpawnAndDestPts!(n::Network, spawns::Vector{Int}, dests::Vector{Int})
    empty!(n.spawns)
    for i in spawns
        n.intersections[i].spawnPoint = true
        push!(n.spawns, n.intersections[i].nodeID)
    end

    empty!(n.dests)
    for i in dests
        n.intersections[i].destPoint = true
        push!(n.dests, n.intersections[i].nodeID)
    end
end

function GetRoadByNodes(n::Network, first::Int, second::Int)::Union{Road,Nothing} #Take two nodes, get road between them
    if first > length(n.intersections) || second > length(n.intersections) #If ID of either nodes not in intersections range give error
        error("Node of of boundries")
    else #Otherwise
        for r in n.roads #For each road
            if r.bNode == first && r.fNode == second #If beggining node equals first and...etc.
                return r #Return the road
            end
        end
    end
    return nothing
end

function GetIntersectByNode(n::Network, id::Int)::Intersection #Takes the network and node ID of the node Intersection it's currently on
    for i in n.intersections
        if i.nodeID == id
            return i #Return the information for the intersection they're on
        end
    end
    return nothing
end

function GetAgentByID(n::Network, id::Int)::Union{Agent,Nothing}
    for i in n.agents
        if i.id == id
            return i
        end
    end
    return nothing
end

function GetIntersectionCoords(n::Network)::DataFrame
    df = DataFrame(first_X = Real[], first_Y = Real[], second_X = Real[], second_Y = Real[])
    for p in GetIntersectionCoords(n)
        push!(df, Dict( :first_X => p[1][1], :first_Y => p[1][2], :second_X => p[2][1], :second_Y => p[2][2]))
    end
    return df
end

function DumpRoadsInfo(s::Simulation)
    for r in s.network.roads
        push!(s.roadInfo, Dict( :iter => s.iter,
                        :bNode => r.bNode,
                        :fNode => r.fNode,
                        :len => r.length,
                        :k => length(r.agents),
                        :k_max => r.capacity,
                        :v => r.curVelocity * 3.6
                        ))
    end
end

function GetNodesInRadius(n::Network, pt::Tuple{Real,Real}, r::Real)::Vector{Int}
    v = Vector{Int}()
    for i in 1:length(n.intersections)
        if EuclideanNorm(pt, (n.intersections[i].posX, n.intersections[i].posY)) <= r
            push!(v, i)
        end
    end
    return v
end

function GetNodesOutsideRadius(n::Network, pt::Tuple{Real,Real}, r::Real)::Vector{Int}
    v = Vector{Int}()
    for i in 1:length(n.intersections)
        if EuclideanNorm(pt, (n.intersections[i].posX, n.intersections[i].posY)) > r
            push!(v, i)
        end
    end
    return v
end

function CanFitAtRoad(a::Agent, r::Road)::Bool
    return length(r.agents) < r.capacity #Check theres room on road for agent
end

function RecalculateRoad!(r::Road) #Set Velocity function
    r.curVelocity = min(lin_k_f_model(length(r.agents), r.capacity, r.vMax, r.vMin), r.vMax)
    #Set the current velocity all cars are going on this road based on length, number of agents on the road (congestion), etc.
    r.ttime = r.length / r.curVelocity
    r.MTS = (r.capacity * r.length * (r.vMax - r.vMin)) / ((r.vMin - r.vMax) * length(r.agents) + r.capacity * r.vMax) ^ 2
end

function GetMTS(r::Road, k::Int)::Real
    if k < 0 || k > r.capacity
        error("Error at GetMTS(). k = $k is beyond range.")
    end
    return (r.capacity * r.length * (r.vMax - r.vMin)) / ((r.vMin - r.vMax) * k + r.capacity * r.vMax) ^ 2
end

function GetRouteDistance(n::Network, r::Vector{Int})::Real
    len = 0.
    for i in 1:(length(r)-1)
        len += GetRoadByNodes(n, r[i], r[i + 1]).length
    end
    return len
end

function SpawnAgents(s::Simulation, dt::Real)
    if s.maxAgents == s.network.agentCntr
        return
    end

    λ = 5.0    #avg number of vehicles per second that appear
    k = minimum([maximum([rand(Distributions.Poisson(λ * dt)), 0]), s.maxAgents - s.network.agentCntr])
    for i in 1:k
        SpawnAgentAtRandom(s.network, s.timeElapsed)
    end
end

function SpawnNumAgents(s::Simulation, num::Int)
    num = min(num, s.maxAgents - s.network.agentCntr)
    for i in 1:num
        SpawnAgentAtRandom(s.network, s.timeElapsed)
    end
end

function SpawnAgentAtRandom(n::Network, time::Real = 0.0)
    start = Int64
    dest = Int64
    create_agent = false

    while create_agent == false
        start = rand(n.spawns)
        dest = rand(n.dests)
        create_agent = IsRoutePossible(n,start,dest)
    end

    σ = 0.1    #standard deviation as a share of expected travel time
    ϵ = 0.1    #starting time glut as a share of travel time

    dt = EstimateTime(n, start, dest)

    arrivT = rand(Distributions.Normal(time + (1.0 + ϵ) * dt, σ * dt))

    n.agentIDmax += 1
    n.agentCntr += 1
    push!(n.agents, Agent(n.agentIDmax, n.intersections[start],n.intersections[dest],n.graph, time, arrivT)) #Randomly spawn an agent within the outer region
end

function RemoveFromRoad!(r::Road, a::Agent)
    a.atRoad = nothing
    deleteat!(r.agents, findall(b -> b == a.id, r.agents))
end

function DestroyAgent(a::Agent, n::Network)
    deleteat!(n.agents, findall(b -> b.id == a.id, n.agents)) #Deletes the agent from the network
    n.agentCntr -= 1 #Global current agents in the network is subtracted by 1
end

function SetWeights!(a::Agent, s::Simulation)
    for r in s.network.roads
        r.ttime = r.length / r.curVelocity
        a.reducedGraph.weights[r.bNode, r.fNode] = r in a.bannedRoads ? Inf : r.ttime * GetVoT(a, s.timeElapsed) + r.length * a.CoF
    end
end

function SetShortestPath!(a::Agent, nw::Network)::Real
    # if a.atNode != nothing
    #     s_star = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID) #Find the shortest path between the node currently at and destination node
    #     dist = s_star.dists[a.atNode.nodeID] #Distance (?)
    #     if dist != Inf #If distance isn't infinity
    #         nextNode = a.atNode.nodeID #Next node equals the ID of the node agent is currently on
    #         path = s_star.parents
    #         empty!(a.bestRoute) #Remove all elements in this array
    #         while nextNode != 0 #While nextNode doesn't equal zero
    #             nextNode = path[nextNode] #Get next road travelled in shortest path
    #             push!(a.bestRoute, nextNode) #Add road to overall shortest path
    #         end
    #         pop!(a.bestRoute)
    #         a.timeEstim = EstimateTime(s.network, a.atNode.nodeID, a.destNode.nodeID)
    #         return a.bestRouteCost = dist
    #     else
    #         return Inf
    #     end
    # end
    # return nothing
    empty!(a.bestRoute)
    nxtNode = a.atNode != nothing ? a.atNode.nodeID : a.atRoad.fNode

    s_star = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID) #Find the shortest path between the node currently at and destination node
    dist = s_star.dists[nxtNode]
    if dist != Inf
        nextNode = nxtNode
        path = s_star.parents
        while nextNode != 0
            nextNode = path[nextNode]
            push!(a.bestRoute, nextNode)
        end
        pop!(a.bestRoute)
        a.timeEstim = EstimateTime(nw, nxtNode, a.destNode.nodeID)
        return a.bestRouteCost = dist
    else
        return Inf
    end
    return nothing
end

function SetAlternatePath!(a::Agent)::Real
    # delNode = a.bestRoute[1]
    # bNode = a.atNode.nodeID
    # fNode = a.destNode.nodeID
    #
    # if a.atNode != nothing
    #     oVal = a.reducedGraph.weights[bNode,delNode]
    #     a.reducedGraph.weights[bNode,delNode] = Inf
    #     s_cross = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID)
    #     dist = s_cross.dists[a.atNode.nodeID]
    #     if dist != Inf
    #         nextNode = a.atNode.nodeID
    #         path = s_cross.parents
    #         empty!(a.alterRoute)
    #         while nextNode != 0
    #             nextNode = path[nextNode]
    #             push!(a.alterRoute, nextNode)
    #         end
    #         pop!(a.alterRoute)
    #         a.reducedGraph.weights[bNode,delNode] = oVal
    #     else
    #         a.reducedGraph.weights[bNode,delNode] = oVal
    #         #AddRegistry("Agent $(a.id) could not find an alterante path at node $(a.atNode.nodeID)", true)
    #     end
    #     return a.alterRouteCost = dist
    # end
    # return nothing
    if length(a.bestRoute) < 2 return Inf end
    empty!(a.alterRoute)

    if a.atNode != nothing
        delNode1 = nxtNode = a.atNode.nodeID
        delNode2 = a.bestRoute[1]
    else
        delNode1 = a.bestRoute[1]
        delNode2 = a.bestRoute[2]
        nxtNode = a.atRoad.fNode
    end

    oVal = a.reducedGraph.weights[delNode1,delNode2]
    a.reducedGraph.weights[delNode1,delNode2] = Inf
    s_cross = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID)
    dist = s_cross.dists[nxtNode]
    if dist != Inf
        nextNode = nxtNode
        path = s_cross.parents
        while nextNode != 0
            nextNode = path[nextNode]
            push!(a.alterRoute, nextNode)
        end
        pop!(a.alterRoute)
    end

    a.reducedGraph.weights[delNode1, delNode2] = oVal
    return a.alterRouteCost = dist
end

#Estimates how much more time agent needs to ed his destination point
function EstimateTime(n::Network, start::Int, dest::Int)::Real
    if start == dest
        return 0
    end

    pth = dijkstra_shortest_paths(n.graph, dest)

    if(pth.dists[start] == Inf)
        return Inf
    #throw(ErrorException("Cannot find route between $(start) and $(dest)."))
    end

    ttot = 0.
    nxt = prev = start

    while (nxt = pth.parents[prev]) != 0
        ttot += GetRoadByNodes(n, prev, nxt).ttime
        prev = nxt
    end
    return ttot
end

function EstimateTimeQuick(n::Network, start::Int, route::Vector{Int})::Real
    ttot = 0.
    prev = start
    for nxt in route
        ttot += GetRoadByNodes(n, prev, nxt).ttime
        prev = nxt
    end

    return ttot
end

function GetVoT(a::Agent, t::Real)::Real
    a.spareTime = (a.requiredArrivalTime - (t + a.timeEstim))
    return minimum([maximum([a.VoT_base - a.VoT_dev * a.spareTime / a.timeEstim, 0.3 * a.VoT_base]), 3 * a.VoT_base])
end

function GetMR(a::Agent, r::Road, t::Real, k::Int)::Real
    return GetMTS(r, k) * GetVoT(a, t)
end

function GetAgentLocation(a::Agent, n::Network)::Union{Tuple{Int,Int,Real}, Nothing}
    if (a.atNode != nothing)
        return a.atNode.nodeID, a.atNode.nodeID, 0.
    elseif a.atRoad != nothing
        return a.atRoad.bNode, a.atRoad.fNode, a.roadPosition
    else
        throw(Exception("Unknown position of agent $(a.id)"))
    end
end

function ReachedIntersection(a::Agent, s::Simulation) #Takes the agent and network
    a.atNode = a.atRoad == nothing ? a.atNode : s.network.intersections[a.atRoad.fNode] #necessary check, as agent spawned is not assigned to any road
    if a.atNode == a.destNode
        AddRegistry("Agent $(a.id) destroyed.") #Add to registry that agent has been destroyed
        RemoveFromRoad!(a.atRoad, a)
        push!(a.travelledRoute,a.atNode.nodeID)
        push!(s.agentsFinished,a)
        a.arrivalTime = s.timeElapsed
        a.totalTravelCost = (a.arrivalTime - a.deployTime) * a.VoT_base + GetRouteDistance(s.network, a.travelledRoute) * a.CoF + GetTotalAuctionBalance(a)
        DestroyAgent(a, s.network) #Destroy the agent
    else
        if isempty(a.travelledRoute)
            push!(a.travelledRoute,a.atNode.nodeID)
        else
            if a.travelledRoute[end] != a.atNode.nodeID
                push!(a.travelledRoute,a.atNode.nodeID)
            end
        end
        SetWeights!(a, s) #Reset weight on edges for agent network
        if SetShortestPath!(a, s.network) == Inf
            println("Agent $(a.id) has a path of infinity. That's not good news!")
            return
        else
            if isempty(a.origRoute)
                cop = deepcopy(a.bestRoute)
                oroute = pushfirst!(cop,a.atNode.nodeID)
                a.origRoute = oroute
            end
            nextRoad = GetRoadByNodes(s.network, a.atNode.nodeID, a.bestRoute[1]) #Get the road agent is turning on

            if (CanFitAtRoad(a, nextRoad)) #Check that he fits on the road
                if(a.atRoad != nothing)
                    RemoveFromRoad!(a.atRoad, a)
                end
                push!(nextRoad.agents, a.id) #Add agents ID onto roads agents array
                a.atRoad = nextRoad #Set agent on new road
                a.roadPosition = 0.0 #Set his position on road to zero
                a.atNode = nothing #Agent no longer at node
            else
                #println("CANT FIT TO ROAD!!!! for agent $(a.id)\nThe road length $(nextRoad.length)\nThe road capacity $(nextRoad.capacity)\nThe number of agents currently on that road $(length(nextRoad.agents))\n")
            end
        end
    end
end

function MakeAction!(a::Agent, sim::Simulation) #Takes an agent ID and simulation
    dt = sim.timeStep

    if a.atNode != nothing
        ReachedIntersection(a, sim)
    else
        a.roadPosition += a.atRoad.curVelocity * dt #The velocity of the road agent is currently on multiplied by the timestep
        a.roadPosition = min(a.roadPosition, a.atRoad.length) #Take the minimum value between the position they are on the road and the length of that road
        #AddRegistry("Agent $(a.id) has travelled $(GetAgentLocation(a, sim.network)[3]) of $(a.atRoad.length) m from $(GetAgentLocation(a, sim.network)[1]) to $(GetAgentLocation(a, sim.network)[2]) at speed: $(a.atRoad.curVelocity*3.6)km/h")
        if a.roadPosition == a.atRoad.length #If their road position is at the end of the road
            ReachedIntersection(a, sim)
        end
    end
    DumpAgentsInfo(a, sim) #Dump info into dataframe
end

function GetTotalAuctionBalance(a::Agent)
    return sum(values(a.moneyReceived)) - sum(values(a.moneySpent))
end

function DumpAgentsInfo(a::Agent, s::Simulation)
    loc = GetAgentLocation(a, s.network)
    if loc == nothing
        return
    else
        (bx, by) = (s.network.intersections[loc[1]].posX, s.network.intersections[loc[1]].posY)
        (fx, fy) = (s.network.intersections[loc[2]].posX, s.network.intersections[loc[2]].posY)
        progress =  a.atRoad == nothing ? 0 : loc[3] / a.atRoad.length
        (x, y) = (bx + progress * (fx - bx), by + progress * (fy - by))

        push!(s.simData, Dict(  :iter => s.iter,
                                :t => s.timeElapsed / 60,
                                :t_est => a.timeEstim / 60,
                                :arrival => a.requiredArrivalTime / 60,
                                :agent => a.id,
                                :node1 => loc[1],
                                :node2 => loc[2],
                                :roadPos => loc[3],
                                :posX => x,
                                :posY => y,
                                :v => a.atRoad != nothing ? a.atRoad.curVelocity * 3.6 : 0,
                                :spare_t => a.spareTime / 60,
                                :vot => GetVoT(a, s.timeElapsed) * 3600,
                                ))
    end
end

function DumpIntersectionsInfo(nw::Network, map::OpenStreetMapX.OSMData)::DataFrame
    df = DataFrame( id = Int[],
                    posX = Real[],
                    posY = Real[],
                    lat = Real[],
                    lon = Real[],
                    ingoing = String[],
                    outgoing = String[]
                    )
    for i in nw.intersections
        push!(df, Dict( :id => i.nodeID,
                        :posX => i.posX,
                        :posY => i.posY,
                        :lat => i.lat,
                        :lon => i.lon,
                        :ingoing => string([r.bNode for r in i.inRoads]),
                        :outgoing => string([r.fNode for r in i.outRoads])
        ))
    end
    return df
end

function DumpFinishedAgents(s::Simulation)::DataFrame
    df = DataFrame( id = Int[],
                    start_node = Int[],
                    end_node = Int[],
                    route = String[],
                    orig_route = String[],
                    banned_roads = String[],
                    deploy_t = Real[],
                    req_arr_t = Real[],
                    arr_t = Real[],
                    travel_t = Real[],
                    travel_dist = Real[],
                    ttc = Real[],
                    vot_base = Real[],
                    money_spent = String[],
                    money_received = String[],
                    money_balance = Real[],
                    auctions = String[]
    )

    for a in s.agentsFinished
        push!(df, Dict( :id => a.id,
                        :start_node => a.origRoute[1],
                        :end_node => a.destNode.nodeID,
                        :route => string(a.travelledRoute),
                        :orig_route => string(a.origRoute),
                        :banned_roads => isempty(a.bannedRoads) ? "None" : string(hcat(getfield.(a.bannedRoads, :bNode), getfield.(a.bannedRoads, :fNode))),
                        :deploy_t => a.deployTime / 60,
                        :req_arr_t => a.requiredArrivalTime / 60,
                        :arr_t => a.arrivalTime / 60,
                        :travel_t => (a.arrivalTime - a.deployTime) / 60,
                        :travel_dist => GetRouteDistance(s.network, a.travelledRoute) / 1000,
                        :ttc => a.totalTravelCost,
                        :vot_base => a.VoT_base * 3600,
                        :money_spent => isempty(a.moneySpent) ? "0" : string(a.moneySpent),
                        :money_received => isempty(a.moneyReceived) ? "0" : string(a.moneyReceived),
                        :money_balance => sum(values(a.moneyReceived)) - sum(values(a.moneySpent)),
                        :auctions => isempty(a.moneySpent) ? "None" : string(keys(a.moneySpent))
        ))
    end
    return df
end

function DumpAuctionsInfo(sim::Simulation)::DataFrame
    s = ""
    df = DataFrame( auction = Int[],
                    b_node = Int[],
                    f_node = Int[],
                    round = Int[],
                    agent = Int[],
                    sale_bid = Real[],
                    buy_bid = Real[],
                    MR = Real[],
                    clearingPrice = Real[],
                    is_winner = Bool[]
                    )
    for au in sim.auctions
        for r in 1:au.rounds
            for a in au.participants
                push!(df, Dict( :auction => au.auctionID,
                                :b_node => au.road.bNode,
                                :f_node => au.road.fNode,
                                :round => r,
                                :agent => a.id,
                                :sale_bid => a.id in keys(au.sBids[r]) ? au.sBids[r][a.id] : Inf,
                                :buy_bid => a.id in keys(au.bBids[r]) ? au.bBids[r][a.id] : 0,
                                :MR => a.id in keys(au.MRs[r]) ? au.MRs[r][a.id] : 0,
                                :clearingPrice => au.clearingPrice[r],
                                :is_winner => a == au.winners[r]
                ))
            end
        end
    end
    return df
    # for au in sim.auctions
    #     s = s * "***Auction: $(au.auctionID):\n-$(length(au.participants)) participants: $(au.participants)\n-rounds: $(au.rounds)\n-time: $(au.time)\n-subject road: $(au.road)\n"
    #     for r in 1:au.rounds
    #         s = s * "\t*Round $r\n\t -Marginal revenues: $(au.MRs[r])\n\t -Buy offers: $(au.bBids[r])\n\t -Sale offers: $(au.sBids[r])\n\t -Clearing price: $(au.clearingPrice[r])\n\t -Winners: $(au.winners)\n"
    #     end
    #     s = s * "\n"
    # end
    # return s
end

function RunSim(s::Simulation, runTime::Int = 0)::Bool
    if runTime > 0
        s.timeMax = s.timeElapsed + runTime
    end

    while s.timeElapsed < s.timeMax
        if !s.isRunning
            return false
        end
        s.iter += 1 #Add new number of iteration
        AddRegistry("[$(round(s.timeElapsed/s.timeMax*100)) %] Iter #$(s.iter), time: $(s.timeElapsed), agents: $(s.network.agentCntr), agents total: $(s.network.agentIDmax)", true)

        for r in s.network.roads #For each road in the network, set the velocity of the road based on congestion, etc.
            RecalculateRoad!(r)
        end

        #Spawn some number of agents
        if s.iter > 1
            SpawnAgents(s, s.timeStep)
        else
            SpawnNumAgents(s, s.initialAgents)
        end

        #For each agent in the network
        for a in s.network.agents
            MakeAction!(a, s) #Do action
        end

        if s.enableAuctions
            if  s.timeElapsed - s.lastAuctionTime >= auctionTimeInterval
                s.lastAuctionTime = s.timeElapsed
                for au in GrabAuctionPlaces(s)
                    CommenceBidding(s, au)
                end
            end
        end

        DumpRoadsInfo(s)

        s.timeElapsed += s.timeStep

        if (s.iter % saveEach) == 0
            SaveSim(s, "sim_stack_4k_10dt_500ini_t=" * string(Int(floor(s.timeElapsed))))
        end

        if s.iter > s.maxIter return false end
    end
    return true
end

function exp_k_f_model(k::Real, k_max::Real, v_max::Real)
    return v_max * exp(- k / k_max)
end

function lin_k_f_model(k::Int, k_max::Int, v_max::Real = 50.0 / 3.6, v_min::Real = 1.0 / 3.6)
    v = (v_max - v_min) * (1.0 - k / k_max) + v_min
    return  v < 0 || v == NaN ? throw(Exception("Velocity less than zero or not a number")) : v
end

function DistanceENU(p1::ENU, p2::ENU)
    return sqrt((p1.east - p2.east)^2 + (p1.north - p2.north)^2 + (p1.up - p2.up)^2)
end

function EuclideanNorm(p1::Tuple{Real,Real}, p2::Tuple{Real,Real})
    return sqrt((p1[1] - p2[1])^2 + (p1[2] - p2[2])^2)
end

function IsRoutePossible(n::Network, startNode::Int64, endNode::Int64)
    route = LightGraphs.dijkstra_shortest_paths(n.graph,endNode)
    dist = route.dists[startNode]
    if dist == Inf
        return false
    return true
    end
end

function GrabAuctionParticipants(nw::Network, r::Road)::Vector{Agent}
    players = Agent[]
    for road in nw.intersections[r.bNode].inRoads
        for a in road.agents
            agent = GetAgentByID(nw, a)
            if SetShortestPath!(agent, nw) > SetAlternatePath!(agent)
                AddRegistry("Oops! The best route has higher cost than alternative. Something is not right.", true)
            end
            if length(agent.bestRoute) >= 2
                #println("road fNode: $(r.fNode), agent $(agent.id): bestroute: $(agent.bestRoute[1]) -> $(agent.bestRoute[2])")
                if agent.bestRoute[2] == r.fNode && agent.isSmart && !(r in agent.bannedRoads)
                    push!(players,agent)
                end
                if agent.bestRoute[2] != r.fNode
                    AddRegistry("Auction road is $(r.bNode)  $(r.fNode), but agent's $(agent.id) bestroute is $(agent.bestRoute[1]) -> $(agent.bestRoute[2])")
                end
            end
        end
    end
    return players
end

function GrabAuctionPlaces(s::Simulation)::Vector{Auction}
    auctions = Vector{Auction}() #Array of auction objects
    for r in s.network.roads #For road in network of roads
        if ( length(r.agents) / r.capacity < auctionMinCongestion || #Congestion is enough
             length(s.network.intersections[r.bNode].inRoads) < 2 || #Two or more in-roads
             length(r.agents) < auctionMinAgentsOnRoad #More than enough agents on the road
             )
            continue
        end
        AddRegistry("Road $(r.bNode) - $(r.fNode) is congested enough.")
        particip = GrabAuctionParticipants(s.network, r)
        buyers,sellers = SplitBuyersAndSellers(s,particip)

        if length(buyers) != 0 && length(sellers) != 0
            #filter out minimal congestion rate threshold
            if length(particip) >= auctionMinParticipants
                s.auctionCntr += 1
                push!(auctions, Auction(s.auctionCntr, particip, r, s.timeElapsed))
            end
        end
    end

    #leave only auctions for roads that are most congested
    if length(auctions) > auctionsAtOnceMax
        sort_k = sort([length(auctions[i].road.agents) / auctions[i].road.capacity for i in 1:length(auctions)])
        min_k = sort_k[end - auctionsAtOnceMax + 1]
        filter!(x -> (length(x.road.agents) / x.road.capacity) >= min_k, auctions)
    end

    for au in auctions
        push!(s.auctions, au)
    end
    AddRegistry("$(length(auctions)) potential auction spot(s) have been selected.", true)
    return auctions
end

function CommenceBidding(s::Simulation, auction::Auction)
    # println(length(auction.participants))
    # StackModelAuction(s, auction)
    #
    # for a in auction.winners
    #     a.reducedGraph.weights[auction.road.bNode, auction.road.fNode] = Inf
    #     push!(a.bannedRoads, auction.road)
    #     SetShortestPath!(a, s.network)
    #     SetAlternatePath!(a)
    # end

    h = HungarianAuction(s,auction)
     if h != nothing
              for agreement in h
                      buyer = agreement[1]
                      seller = agreement[2]
                      price = agreement[3]
                      println("Agent #$(agreement[2].id) is bought off by Agent #$(agreement[1].id) for \$$(agreement[3])")
                      seller.reducedGraph.weights[auction.road.bNode, auction.road.fNode] = Inf
                      push!(seller.bannedRoads, auction.road)
                      SetShortestPath!(seller, s.network)
                      SetAlternatePath!(seller)
                      push!(buyer.moneySpent, auction.auctionID => price )
                      push!(seller.moneyReceived, auction.auctionID => price)
                      auction.payments[buyer.id,seller.id] = price
                      push!(auction.sBids, Dict(seller.id => price))
                      push!(auction.bBids, Dict(buyer.id => price))
                      auction.rounds = 1
              end
      end
end

"-------------------------------------------------------------------------"

function HungarianAuction(s::Simulation, auction::Auction)
    final_sellers = Tuple{Agent,Float64}[]
    buyers, sellers = SplitBuyersAndSellers(s,auction.participants)
    if length(buyers) != 0 && length(sellers) != 0
        for seller in sellers
            push!(final_sellers,(seller,CalculateCostDifference(seller)))
        end

        final_buyers = CreateBuyersMatrix(buyers,sellers,s,auction.road)
        payoff_matrix = CreatePayoffMatrix(final_buyers,final_sellers)
        VA = VectorOfArray(payoff_matrix)
        arr = convert(Array,VA)

        FixNegatives(arr)
        h = Hungarian(arr)
        Calculate(h)

        return_list = Tuple{Agent,Agent,Float64}[]
        for result in get_results(h)
            if arr[result[1],result[2]] != 0
                price = final_sellers[result[1]][2]
                found_seller = final_sellers[result[1]][1]
                found_buyer = buyers[result[2]]
                push!(return_list,(found_buyer,found_seller,price))
            end
        end


        return return_list
    else
        println("There were $(length(buyers)) number of buyers")
        println("There were $(length(sellers)) number of sellers")
        println("Therefore auction not possible")

    end

    return nothing

end

function CalculateCostDifference(seller::Agent)
    value = round(seller.alterRouteCost - seller.bestRouteCost,digits = 2)
    value = value > 0.00 ? value : 0.01
    return value

end

function SplitBuyersAndSellers(s::Simulation, agents::Array{Agent})
    buyers = Agent[]
    sellers = Agent[]
    for a in agents
        if (s.timeElapsed + a.timeEstim) >= a.requiredArrivalTime
            push!(buyers,a)
        else
            push!(sellers,a)
        end
    end
    return buyers, sellers
end

function CalculateEvaluations(nw::Network, subj_road::Road, buyer::Agent, seller::Agent, t::Real)
    # value = rand(Uniform(0.01,0.2))
    # value = round(value,digits = 2)

    if buyer.atRoad != nothing
        t_buyer = buyer.atRoad.ttime * (buyer.atRoad.length - buyer.roadPosition) / buyer.atRoad.length
    else
        nxtRoad = GetRoadByNodes(nw, buyer.atNode.nodeID, buyer.bestRoute[1])
        t_buyer = nxtRoad.ttime
    end

    if seller.atRoad != nothing
        t_seller = seller.atRoad.ttime * (seller.atRoad.length - seller.roadPosition) / seller.atRoad.length
    else
        nxtRoad = GetRoadByNodes(nw, seller.atNode.nodeID, seller.bestRoute[1])
        t_seller = nxtRoad.ttime
    end

    t_both = 0
    if t_buyer < t_seller
        return 0
    else
        dt = t_buyer - t_seller
        dist_seller = subj_road.curVelocity * dt
        if dist_seller >= subj_road.length
            return 0
        else
            t_both = (subj_road.length - dist_seller) / subj_road.curVelocity
            fract = t_both / (subj_road.ttime)

            if fract > 1 || fract < 0
                println("Fraction time $fract is not between 0 and 1")
            end
        end
    end

    buyer_mr = GetMR(buyer, subj_road, t, length(subj_road.agents))
    value =  buyer_mr * fract
#    println("Agent $(buyer.id) values agent $(seller.id) at $value, with his MR = $buyer_mr. They will spend $t_both s on the same road out of $(subj_road.ttime) s needed for buyer to traverse road.")
    return value
end

function CreateBuyersMatrix(buyers::Array{Agent},sellers::Array{Agent},s::Simulation, r::Road)
    final_buyers = Array{Float64}[]
    for buyer in buyers
        templist = Float64[]
        for seller in sellers
            push!(templist,CalculateEvaluations(s.network, r, buyer, seller, s.timeElapsed))
        end
        push!(final_buyers,templist)
    end
    return final_buyers
end

function CreatePayoffMatrix(buyers::Array{Array{Float64}},sellers::Array{Tuple{Agent,Float64}})
    payoff_matrix = Array{Float64}[]
    for agent_values in buyers
            temp_list = Float64[]
            for i = 1:length(sellers)
                    push!(temp_list, agent_values[i] - sellers[i][2] > 0.00 ? agent_values[i] - sellers[i][2] : 0.00)
            end
            push!(payoff_matrix,temp_list)
    end

    return payoff_matrix
end

"-------------------------------------------------------------------------"

function StackModelAuction(s::Simulation, au::Auction)
    au.rounds = 1
    remaining_agents = deepcopy(au.participants)

    for a in au.participants
        push!(a.moneySpent, au.auctionID => 0.)
    end

    while true
        #recalculate MRs and sale offers for all the reamining participants
        println("Auction: $(au.auctionID)")
        all_sBids = Dict([(a.id, (a.alterRouteCost - a.bestRouteCost)) for a in remaining_agents[getfield.(remaining_agents, :alterRouteCost) .> getfield.(remaining_agents, :bestRouteCost)]])
        all_MRs = Dict([(a.id, GetMR(a, au.road, au.time, maximum([length(au.road.agents) + 1 - au.rounds, 1]))) for a in remaining_agents])

        #sort offers in ascending order
        MRsOrd = sort(collect(all_MRs), by = x -> x[2])
        sBidsOrd = sort(collect(all_sBids), by = x -> x[2])

        if length(MRsOrd) < 2 || length(sBidsOrd) < 1
            @goto stop
        end

        minS = sBidsOrd[1][2]      #lowest sale offer
        seller = GetAgentByID(s.network, sBidsOrd[1][1])

        filter!(x -> x[1] != sBidsOrd[1][1], MRsOrd)   #delete buyer with the lowest sale offer as he will be bought out in the first place

        push!(au.MRs, Dict(MRsOrd))
        push!(au.sBids, Dict(sBidsOrd))

        N = length(MRsOrd)

        ΔMRs = [i == 1 ? MRsOrd[1][2] : MRsOrd[i][2] - MRsOrd[i-1][2] for i in 1:N]

        tot = 0
        k1 = 0
        α = 0.0
        for k in 1:N
            tot += ΔMRs[k] * (N - k + 1)
            if tot >= minS
                k1 = k
                break
            end
        end
        if k1 == 0
            @label stop
            AddRegistry("No more buy-offs possible! Auction $(au.auctionID) stopped at round $(au.rounds) with $(length(remaining_agents)) participants.", true)
            AddRegistry("Auction $(au.auctionID) round $(au.rounds): total buyers' bid $(tot), lowest sale bid: $(minS) exceeded by $(100*(minS/tot-1))%", true)
            au.rounds -= 1
            return
        else
            last = ΔMRs[k1] * (N - k1 + 1)
            tot_minus_last = tot - last
            α = (minS - tot_minus_last) / last
            println("α: $(α)")

            maxOff = (k1 == 1 ? 0 : MRsOrd[k1 - 1][2]) + α * MRsOrd[k1][2]

            push!(au.clearingPrice, minS)
            push!(au.bBids, Dict([(MRsOrd[i][1], minimum([MRsOrd[i][2], maxOff])) for i in 1:length(MRsOrd)]))

            for b in keys(au.bBids[end])
                au.payments[b, sBidsOrd[1][1]] = au.bBids[end][b]
            end

            push!(seller.moneyReceived, au.auctionID => minS)
            push!(au.winners, seller)

            for b in keys(au.bBids[end])
                GetAgentByID(s.network, b).moneySpent[au.auctionID] += au.bBids[end][b]
            end

            filter!(x -> x.id != sBidsOrd[1][1], remaining_agents) #agent with the lowest sale offer has been bought off, thus shall be removed from the auction

            println("Auction $(au.auctionID) round $(au.rounds): maximal offer $(maxOff) submitted by $(N - k1 + 1) buyer(s).")
        end

        au.rounds += 1
        if au.rounds > length(au.participants) - 1
            AddRegistry("Oops! Something's wrong with your while loop at auction $(au.auctionID).", true)
            break
        end
    end
end

function SaveSim(sim::Simulation, file::String)::Bool
    f = open("/Users/arashdehghan/Desktop/RouteBidModel/results/" * file * ".sim", "w")
    serialize(f, sim)
    close(f)
    AddRegistry("File successfully saved as " * "/Users/arashdehghan/Desktop/RouteBidModel/results/" * file * ".sim", true)
    return true
end

function LoadSim(file::String)::Simulation
    f = open("/Users/arashdehghan/Desktop/RouteBidModel/results/" * file * ".sim", "r")
    sim = deserialize(f)
    close(f)

    return sim
end


"---------------------------------------------------------------------------------"

function Calculate(h::Hungarian)
    result_matrix = copy(h.cost_matrix)
    # Step 1: Subtract row mins from each row.
    result_matrix = SubtractRows(result_matrix,h.matrix_size)

    # Step 2: Subtract column mins from each column
    result_matrix = SubtractColumns(result_matrix,h.matrix_size)

    # Step 3: Use minimum number of lines to cover all zeros in the matrix.

    total_covered = 0
    while total_covered < h.matrix_size
            cover_zeros = CoverZeros(result_matrix)
            _calculate(cover_zeros)
            cover_zeros.covered_rows = setdiff(collect(1:h.matrix_size),cover_zeros.marked_rows)
            cover_zeros.covered_columns = cover_zeros.marked_columns
            covered_rows = get_covered_rows(cover_zeros)
            covered_columns = get_covered_columns(cover_zeros)
            total_covered = length(covered_rows) + length(covered_columns)

        #     if the total covered rows+columns is not equal to the matrix size then adjust it by min uncovered num (m).
            if total_covered < h.matrix_size
                    result_matrix = _adjust_matrix_by_min_uncovered_num(h,result_matrix, covered_rows, covered_columns)
            end
    end

    # Step 4: Starting with the top row, work your way downwards as you make assignments.
    # Find single zeros in rows or columns.
    # Add them to final result and remove them and their associated row/column from the matrix.

    expected_results = min(h.max_columns, h.max_rows)
    zero_locations = GrabZeroLocations(result_matrix)

    while length(h.results) != expected_results
            # If number of zeros in the matrix is zero before finding all the results then an error has occurred.
            if any(x->x==true, zero_locations) == false
                    println("Unable to find results. Algorithm failed.")
                    exit()
            end
            # Find results and mark rows and columns for deletion
            matched_rows, matched_columns = _find_matches(zero_locations,h)

            # Make arbitrary selection
            total_matched = length(matched_rows) + length(matched_columns)
            if total_matched == 0
                    matched_rows, matched_columns = _select_arbitrary_match(zero_locations)
            end

            for row in matched_rows
                    for col = 1:length(zero_locations[row,:])
                            zero_locations[row,col] = false
                    end
            end

            for col in matched_columns
                    for row = 1:length(zero_locations[:,col])
                            zero_locations[row,col] = false
                    end
            end

            _set_results(zip(matched_rows,matched_columns),h)

    end

    # Calculate total potential
    val = 0
    for value in h.results
            val += h.original_matrix[value[1],value[2]]
            h.totalPotential = val
    end


end

function get_results(h::Hungarian)
        """Get results after calculation."""
        return h.results
end

function get_total_potential(h::Hungarian)
        """Returns expected value after calculation."""
        return h.totalPotential
end

function _set_results(result_lists::Base.Iterators.Zip, h::Hungarian)
        """Set results during calculation."""
        # Check if results values are out of bound from input matrix (because of matrix being padded).
        # Add results to results list.
        for result in result_lists
                row, column = result
                if row <= h.max_rows && column <= h.max_columns
                        new_result = (Int64(row), Int64(column))
                        push!(h.results,new_result)
                end
        end
end

function _select_arbitrary_match(zero_locations::Array{Bool})
        """Selects row column combination with minimum number of zeros in it."""
        # Count number of zeros in row and column combinations
        rows, columns = WhereAll(zero_locations)
        zero_count = []
        for index = 1:length(rows)
                total_zeros = sum(zero_locations[rows[index],:]) + sum(zero_locations[:,columns[index]])
                push!(zero_count,total_zeros)
        end

        # Get the row column combination with the minimum number of zeros.
        indices = findall(x->x==minimum(zero_count), zero_count)[1]
        row = [rows[indices]]
        column = [columns[indices]]

        return row, column


end

function WhereAll(m::Array{Bool})
        rows = Int64[]
        cols = Int64[]
        for row = 1:size(m)[1]
                for col = 1:size(m)[2]
                        if m[row,col] == true
                                push!(rows,row)
                                push!(cols,col)
                        end
                end
        end
        return rows,cols
end

function _find_matches(zero_locations::Array{Bool},h::Hungarian)
        """Returns rows and columns with matches in them."""
        marked_rows = Int64[];
        marked_columns = Int64[];
        # Mark rows and columns with matches
        # Iterate over rows
        for index = 1:size(zero_locations)[1]
                row_index = [index]
                if sum(zero_locations[index,:]) == 1
                        column_index = Where(zero_locations[index,:])
                        marked_rows, marked_columns = _mark_rows_and_columns(marked_rows,marked_columns,row_index,column_index)
                end
        end

        for index = 1:size(zero_locations)[1]
                column_index = [index]
                if sum(zero_locations[:,index]) == 1
                        row_index = Where(zero_locations[:,index])
                        marked_rows, marked_columns = _mark_rows_and_columns(marked_rows,marked_columns,row_index,column_index)
                end
        end

        return marked_rows, marked_columns
end

function _mark_rows_and_columns(marked_rows::Array{Int64},marked_columns::Array{Int64},row_index::Array{Int64},column_index::Array{Int64})
        """Check if column or row is marked. If not marked then mark it."""
        new_marked_rows = marked_rows
        new_marked_columns = marked_columns
        if (any(x->x==true, Equalizer(marked_rows,row_index)) == false) && (any(x->x==true, Equalizer(marked_columns,column_index)) == false)
                splice!(marked_rows,length(marked_rows)+1:length(marked_rows),row_index)
                new_marked_rows = marked_rows
                splice!(marked_columns,length(marked_columns)+1:length(marked_columns),column_index)
                new_marked_columns = marked_columns
        end
        return new_marked_rows,new_marked_columns
end

function Equalizer(a::Array{Int64},b::Array{Int64})::Array{Bool}
        temp = Bool[]
        for val in a
                for val2 in b
                        push!(temp,val == val2)
                end
        end
        return temp
end

function _adjust_matrix_by_min_uncovered_num(h::Hungarian,result_matrix::Array{Float64}, covered_rows::Array{Int64}, covered_columns::Array{Int64})
        """Subtract m from every uncovered number and add m to every element covered with two lines."""
        # Calculate minimum uncovered number (m)
        elements = []
        for row_index = 1:h.matrix_size
                if any(x->x==row_index, covered_rows) == false
                        for index = 1:length(result_matrix[row_index,:])
                                if any(x->x==index, covered_columns) == false
                                        push!(elements,result_matrix[row_index,index])
                                end

                        end
                end
        end
        min_uncovered_num = minimum(elements)

        # Add m to every covered element
        adjusted_matrix = result_matrix

        for row in covered_rows
                for col = 1:length(adjusted_matrix[row,:])
                        adjusted_matrix[row,col] += min_uncovered_num
                end
        end

        for column in covered_columns
                for row = 1:length(adjusted_matrix[:,column])
                        adjusted_matrix[row,column] += min_uncovered_num
                end
        end

        # Subtract m from every element
        m_matrix = ones(Int64,(size(adjusted_matrix))) * min_uncovered_num
        adjusted_matrix -= m_matrix
        return adjusted_matrix

end



function _calculate(c::CoverZeros)
        while true
                c.marked_rows = Int64[]
                c.marked_columns = Int64[]

                # Mark all rows in which no choice has been made.
                for row = 1:c.mshape[1]
                        if any(x->x==true, c.choices[row,:]) == false
                                push!(c.marked_rows,row)
                        end
                end

                # If no marked rows then finish.
                if isempty(c.marked_rows)
                        return true
                end

                # Mark all columns not already marked which have zeros in marked rows.
                num_marked_columns = _mark_new_columns_with_zeros_in_marked_rows(c)

                # If no new marked columns then finish.
                if num_marked_columns == 0
                        return true
                end
                # While there is some choice in every marked column.
                while _choice_in_all_marked_columns(c)
                        # Some Choice in every marked column.

                        # Mark all rows not already marked which have choices in marked columns.
                        num_marked_rows = _mark_new_rows_with_choices_in_marked_columns(c)

                        # If no new marks then Finish.
                        if num_marked_rows == 0
                                return true
                        end

                        # Mark all columns not already marked which have zeros in marked rows.
                        num_marked_columns = _mark_new_columns_with_zeros_in_marked_rows(c)

                        # If no new marked columns then finish.
                        if num_marked_columns == 0
                                return true
                        end

                end
                # No choice in one or more marked columns.
                # Find a marked column that does not have a choice.
                choice_column_index = _find_marked_column_without_choice(c)
                while choice_column_index != nothing
                        # Find a zero in the column indexed that does not have a row with a choice.
                        choice_row_index = _find_row_without_choice(choice_column_index,c)

                        # Check if an available row was found.
                        new_choice_column_index = nothing
                        if choice_row_index == nothing
                                # Find a good row to accomodate swap. Find its column pair.
                                choice_row_index, new_choice_column_index = _find_best_choice_row_and_new_column(choice_column_index,c)

                                # Delete old choice.
                                c.choices[choice_row_index,new_choice_column_index] = false
                        end
                        # Set zero to choice.
                        c.choices[choice_row_index,choice_column_index] = true

                        # Loop again if choice is added to a row with a choice already in it.
                        choice_column_index = new_choice_column_index
                end
        end




end

function _mark_new_columns_with_zeros_in_marked_rows(c::CoverZeros)
        """Mark all columns not already marked which have zeros in marked rows."""
        num_marked_columns = 0
        for col = 1:c.mshape[1]
                if any(x->x==col, c.marked_columns) == false
                        if any(x->x==true, c.zero_locations[:,col])
                                row_indices = Where(c.zero_locations[:,col])
                                zeros_in_marked_rows = !isempty(intersect(c.marked_rows,row_indices))
                                if zeros_in_marked_rows
                                        push!(c.marked_columns,col)
                                        num_marked_columns += 1
                                end
                        end
                end
        end
        return num_marked_columns
end

function _choice_in_all_marked_columns(c::CoverZeros)
        """Return Boolean True if there is a choice in all marked columns. Returns boolean False otherwise."""
        for column_index in c.marked_columns
                if any(x->x==true, c.choices[:,column_index]) == false
                        return false
                end
        end
        return true

end

function _mark_new_rows_with_choices_in_marked_columns(c::CoverZeros)
        """Mark all rows not already marked which have choices in marked columns."""
        num_marked_rows = 0
        for row = 1:c.mshape[1]
                if any(x->x==row, c.marked_rows) == false
                        if any(x->x==true, c.choices[row,:])
                                column_index = Where(c.choices[row,:])
                                if column_index[1] in c.marked_columns
                                        push!(c.marked_rows,row)
                                        num_marked_rows += 1
                                end
                        end

                end
        end
        return num_marked_rows

end

function _find_marked_column_without_choice(c::CoverZeros)
        """Find a marked column that does not have a choice."""
        for column_index in c.marked_columns
                if any(x->x==true, c.choices[:,column_index]) == false
                        return column_index
                end
        end
        println("Could not find a column without a choice. Failed to cover matrix zeros. Algorithm has failed.")
        exit()
end

function _find_row_without_choice(choice_column_index::Int64,c::CoverZeros)
        """Find a row without a choice in it for the column indexed. If a row does not exist then return None."""
        row_indices = Where(c.zero_locations[:,choice_column_index])
        for row_index in row_indices
                if any(x->x==true, c.choices[row_index,:]) == false
                        return row_index
                end
        end
        # All rows have choices. Return None.
        return nothing
end

function _find_best_choice_row_and_new_column(choice_column_index::Int64, c::CoverZeros)
        """
        Find a row index to use for the choice so that the column that needs to be changed is optimal.
        Return a random row and column if unable to find an optimal selection.
        """
        row_indices = Where(c.zero_locations[:,choice_column_index])
        for row_index in row_indices
                column_indices = Where(c.choices[row_index,:])
                column_index = column_indices[1]
                if _find_row_without_choice(column_index,c) != nothing
                        return row_index, column_index
                end
        end

        # Cannot find optimal row and column. Return a random row and column.
        shuffle!(row_indices)
        column_index = Where(c.choices[row_indices[1],:])
        return row_indices[1], column_index[1]
end

function SubtractRows(matrix::Array{Float64},matrix_size::Int64)::Array{Float64}
        for row = 1:matrix_size
                row_min = minimum(matrix[row,:])
                for col = 1:matrix_size
                        matrix[row,col] -= row_min
                end
        end
        return matrix
end

function SubtractColumns(matrix::Array{Float64}, matrix_size::Int64)::Array{Float64}
        for col = 1:matrix_size
                col_min = minimum(matrix[:,col])
                for row = 1:matrix_size
                        matrix[row,col] -= col_min
                end
        end
        return matrix
end

function get_covered_rows(c::CoverZeros)
        """Return list of covered rows."""
        return c.covered_rows
end

function get_covered_columns(c::CoverZeros)
        """Return list of covered columns."""
        return c.covered_columns
end

function Where(m::Array{Bool})::Array{Int64}
        temp_list = Int64[]
        for ind = 1:length(m)
                if m[ind] == true
                        push!(temp_list,ind)
                end
        end
        return temp_list
end

function FixNegatives(m::Array{Float64})
        min_value = minimum(m)
        if min_value < 0
                count = 1
                for value in m
                        m[count] += abs(min_value)
                        count+=1
                end
        end
end

"---------------------------------------------------------------------------------"
