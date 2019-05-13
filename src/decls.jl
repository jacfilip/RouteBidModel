module Decls

using LightGraphs, SimpleWeightedGraphs
using OpenStreetMapX
using Compose
using DataFrames
using Distributions
using SparseArrays
using Conda
using PyCall

export Network
export Simulation
export Agent
export Road
export Intersection
export SetLL
export MakeAction!
export RunSim
export SetSpawnAndDestPts!
export SpawnAgentAtRandom

using Random
Random.seed!(0);

bigNum = 1000000

agentCntr = 0
agentIDmax = 0

headway = 0.5
avgCarLen = 5.

auctionTimeInterval = 60.0
auctionMinCongestion = 0.8
auctionMinParticipants = 10
auctionMinAgentsOnRoad = 10
auctionsAtOnceMax = 5

auctionCntr = 0

muteRegistering = false
simLog = Vector{String}()

path = "maps"
file = "buffaloF.osm"

function AddRegistry(msg::String, prompt::Bool = false)
    if muteRegistering return end

    push!(simLog, msg)
    if prompt
        println(msg)
    end
end

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
    r.vMax = vel;       #ToDo: retrieve from osm
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
    BorS::Int64
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

    Agent(start::Intersection, dest::Intersection, graph::SimpleWeightedDiGraph, deployTime::Real, arrivTime::Real) = (
        a = new();
        a.atNode = start;
        a.destNode = dest;
        a.reducedGraph = deepcopy(graph);

        global agentCntr += 1;
        global agentIDmax += 1;
        a.id = agentIDmax;
        a.roadPosition = 0.0;
        a.VoT_base = maximum([rand(Distributions.Normal(24.52/60.0/60.0, 3.0/60.0/60.0)), 0.0]);
        a.VoT_dev =  1.0 / (rand() * 8.0 + 2.0);
        a.CoF = 0.15e-3;         #ToDo: Check and draw value
        a.carLength = 3.0;      #m ToDo: Draw value
        a.vMax = 120.0 / 3.6;         #m/s
        a.bestRoute = Vector{Int}();
        a.alterRoute = Vector{Int}();
        a.origRoute = Vector{Int}();
        a.travelledRoute = Vector{Int}();
        a.atRoad = nothing;
        a.BorS = rand(Categorical([0.5,0.5]),1)[1];
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
    Network(g::SimpleWeightedDiGraph, coords::Vector{Tuple{Float64,Float64,Float64}}, mdata::MapData) = (
            n = new();
            n.graph = deepcopy(g);
            n.numRoads = 0;
            n.spawns = Vector{Intersection}();
            n.dests = Vector{Intersection}();
            n.agents = Vector{Agent}();
            n.mapData = mdata;
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

    Auction(participants::Vector{Agent}, road::Road, time::Real) = (
        au = new();
        au.participants = participants;
        au.road = road;
        global auctionCntr += 1;
        au.auctionID = auctionCntr;
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

function GetTimeStep(s::Simulation)::Real
    if s.timeStep != 0
        return s.timeStep
    elseif length(s.timeStepVar) > 0
        return s.timeStepVar[end]
    else
        return 0
    end

end

function SetTimeStep!(s::Simulation)::Real
    if s.timeStep != 0
        push!(s.timeStepVar, s.timeStep)
    else
        t_min = Inf
         for r in s.network.roads
             if !isempty(r.agents)
                if (t = (r.length - GetAgentByID(s.network, r.agents[1]).roadPosition) / r.curVelocity) < t_min
                    t_min = t
                end
             end
         end

         t_min = t_min == Inf ? 0.0 : t_min

         if t_min < 0
            throw(ErrorException("Error, simulation step is negative!"))
        end
        push!(s.timeStepVar, maximum([t_min, s.dt_min]))
    end
    #AddRegistry("Time step: $(s.timeStepVar[s.iter]) at iter $(s.iter)")
    return s.timeStepVar[end]
end

function InitNetwork!(n::Network, coords::Vector{Tuple{Float64,Float64,Float64}})
    global agentIDmax = 0
    global agentCntr = 0
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

# function GetIntersectionCoords(n::Network)::Vector{Tuple{Tuple{Real,Real},Tuple{Real,Real}}}
#     tups = (i -> (i.posX, i.posY)).(n.intersections)
#
#     return pts = [((tups[r.bNode][1], tups[r.bNode][2]), (tups[r.fNode][1], tups[r.fNode][2])) for r in n.roads]
# end

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
    if s.maxAgents == agentCntr
        return
    end

    λ = 5.0    #avg number of vehicles per second that appear
    k = minimum([maximum([rand(Distributions.Poisson(λ * dt)), 0]), s.maxAgents - agentCntr])
    for i in 1:k
        SpawnAgentAtRandom(s.network, s.timeElapsed)
    end
end

function SpawnNumAgents(s::Simulation, num::Int)
    num = min(num, s.maxAgents - agentCntr)
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

    push!(n.agents, Agent(n.intersections[start],n.intersections[dest],n.graph, time, arrivT)) #Randomly spawn an agent within the outer region
    AddRegistry("Agent #$(agentIDmax) has been created.") #Adds to registry that agent has been created
end

function RemoveFromRoad!(r::Road, a::Agent)
    a.atRoad = nothing
    deleteat!(r.agents, findall(b -> b == a.id, r.agents))
end

function DestroyAgent(a::Agent, n::Network)
    deleteat!(n.agents, findall(b -> b.id == a.id, n.agents)) #Deletes the agent from the network
    global agentCntr -= 1 #Global current agents in the network is subtracted by 1
end

function SetWeights!(a::Agent, s::Simulation)
    for r in s.network.roads
        r.ttime = r.length / r.curVelocity
        a.reducedGraph.weights[r.bNode, r.fNode] = r in a.bannedRoads ? Inf : r.ttime * GetVoT(a, s.timeElapsed) + r.length * a.CoF
    end
end

function SetShortestPath!(a::Agent, s::Simulation)::Real #When would this return INFINITY??
    if a.atNode != nothing #If the agent is at the
        s_star = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID) #Find the shortest path between the node currently at and destination node
        dist = s_star.dists[a.atNode.nodeID] #Distance (?)
        if dist != Inf #If distance isn't infinity
            nextNode = a.atNode.nodeID #Next node equals the ID of the node agent is currently on
            path = s_star.parents
            empty!(a.bestRoute) #Remove all elements in this array
            while nextNode != 0 #While nextNode doesn't equal zero
                nextNode = path[nextNode] #Get next road travelled in shortest path
                push!(a.bestRoute, nextNode) #Add road to overall shortest path
            end
            pop!(a.bestRoute)
            a.timeEstim = EstimateTime(s.network, a.atNode.nodeID, a.destNode.nodeID)
            return a.bestRouteCost = dist
        else
            return Inf
        end
    end
    return nothing
end

function SetAlternatePath!(a::Agent)::Real
    delNode = a.bestRoute[1]
    bNode = a.atNode.nodeID
    fNode = a.destNode.nodeID

    if a.atNode != nothing
        oVal = a.reducedGraph.weights[bNode,delNode]
        a.reducedGraph.weights[bNode,delNode] = Inf
        s_cross = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID)
        dist = s_cross.dists[a.atNode.nodeID]
        if dist != Inf
            nextNode = a.atNode.nodeID
            path = s_cross.parents
            empty!(a.alterRoute)
            while nextNode != 0
                nextNode = path[nextNode]
                push!(a.alterRoute, nextNode)
            end
            pop!(a.alterRoute)
            a.reducedGraph.weights[bNode,delNode] = oVal
            return a.alterRouteCost = dist
        else
            a.reducedGraph.weights[bNode,delNode] = oVal
            #AddRegistry("Agent $(a.id) could not find an alterante path at node $(a.atNode.nodeID)", true)
            return Inf
        end
    end
    return nothing
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

    a.atNode = a.atRoad == nothing ? a.atNode : s.network.intersections[a.atRoad.fNode]
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
        if SetShortestPath!(a, s) == Inf
            println("Agent $(a.id) has a path of infinity. That's not good news!")
            return
        else
            if isempty(a.origRoute)
                cop = deepcopy(a.bestRoute)
                oroute = pushfirst!(cop,a.atNode.nodeID)
                a.origRoute = oroute
            end

            SetAlternatePath!(a)#,a.bestRoute[1],a.atNode.nodeID,a.destNode.nodeID)
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
    dt = GetTimeStep(sim)

    if a.atNode != nothing
        ReachedIntersection(a, sim)
    else
        a.roadPosition += a.atRoad.curVelocity * dt #The velocity of the road agent is currently on multiplied by the timestep
        a.roadPosition = min(a.roadPosition, a.atRoad.length) #Take the minimum value between the position they are on the road and the length of that road
        #AddRegistry("Agent $(a.id) has travelled $(GetAgentLocation(a, sim.network)[3]) of $(a.atRoad.length) m from $(GetAgentLocation(a, sim.network)[1]) to $(GetAgentLocation(a, sim.network)[2]) at speed: $(a.atRoad.curVelocity*3.6)km/h")
        if a.roadPosition == a.atRoad.length #If their road position is at the end of the road
            ReachedIntersection(a, sim)
            # a.atNode = GetIntersectByNode(sim.network, a.atRoad.fNode) #Set the node I'm currently at
            # RemoveFromRoad!(a.atRoad, a) #Remove the agent from the road they were on
        end
    end

    # if a.atNode != nothing
    #     ReachedIntersection(a, sim)
    # end

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
                    money_spent = Real[],
                    money_received = Real[],
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
                        :travel_t => (a.arrivTime - a.deployTime) / 60,
                        :travel_dist => GetRouteDistance(s.network, a.travelledRoute),
                        :ttc => a.totalTravelCost,
                        :vot_base => a.VoT_base,
                        :money_spent => string(a.moneySpent),
                        :money_received => string(a.moneyReceived),
                        :money_balance => sum(values(a.moneyReceived)) - sum(values(a.moneySpent)),
                        :auctions => isempty(a.moneySpent) ? "None" : string(keys(a.moneySpent))
        ))
    end
    return df
end

function DumpAuctionsInfo(sim::Simulation)::String
    s = ""
    for au in sim.auctions
        s = s * "***Auction: $(au.auctionID):\n-$(length(au.participants)) participants: $(au.participants)\n-rounds: $(au.rounds)\n-time: $(au.time)\n-subject road: $(au.road)\n"
        for r in 1:au.rounds
            s = s * "\t*Round $r\n\t -Marginal revenues: $(au.MRs[r])\n\t -Buy offers: $(au.bBids[r])\n\t -Sale offers: $(au.sBids[r])\n\t -Clearing price: $(au.clearingPrice[r])\n\t -Winners: $(au.winners)\n"
        end
        s = s * "\n"
    end
    return s
end

function RunSim(s::Simulation)::Bool
    while s.timeElapsed < s.timeMax
        if !s.isRunning
            return false
        end
        s.iter += 1 #Add new number of iteration
        AddRegistry("[$(round(s.timeElapsed/s.timeMax*100))%] Iter #$(s.iter), time: $(s.timeElapsed), #agents: $(agentCntr), agents total: $agentIDmax", true)

        for r in s.network.roads #For each road in the network, set the velocity of the road based on congestion, etc.
            RecalculateRoad!(r)
        end

        #Spawn some number of agents
        if s.iter > 1
            SpawnAgents(s, GetTimeStep(s))
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

        s.timeElapsed += SetTimeStep!(s)

        if s.iter > s.maxIter return false end
    end
    # OVAGraph("/Users/arashdehghan/Desktop/RouteBidModel/maps/","buffaloF.osm",s.agentsFinished[1])
    # GraphAgents("/Users/arashdehghan/Desktop/RouteBidModel/maps/","buffaloF.osm",s.agentsFinished)
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
            if length(agent.bestRoute) >=2
                if agent.bestRoute[2] == r.fNode && agent.isSmart && !(road in agent.bannedRoads)
                    push!(players,agent)
                end
            end
        end
    end
    return players
end

function GrabAuctionPlaces(s::Simulation)::Vector{Auction}
    auctions = Vector{Auction}()
    for r in s.network.roads
        if ( length(r.agents) / r.capacity < auctionMinCongestion ||
             length(s.network.intersections[r.bNode].inRoads) < 2 ||
             length(r.agents) < auctionMinAgentsOnRoad
             )
            continue
        end

        particip = GrabAuctionParticipants(s.network, r)

        #filter out minimal congestion rate threshold
        if length(particip) >= auctionMinParticipants
            push!(auctions, Auction(particip, r, s.timeElapsed))
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
    return auctions
end

function CommenceBidding(s::Simulation, auction::Auction)
    StackModelAuction(s, auction)

    for a in auction.winners
        a.reducedGraph.weights[auction.road.bNode, auction.road.fNode] = Inf
        push!(a.bannedRoads, auction.road)
    end
end

function StackModelAuction(s::Simulation, au::Auction)
    au.rounds = 1
    #push!(au.sBids, Dict([(a.id, (a.alterRouteCost - a.bestRouteCost)) for a in au.participants[getfield.(au.participants, :alterRouteCost) .> getfield.(au.participants, :bestRouteCost)]]))
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

            push!(s.network.agents[sBidsOrd[1][1]].moneyReceived, au.auctionID => minS)
            push!(au.winners, s.network.agents[sBidsOrd[1][1]])

            for b in keys(au.bBids[end])
                s.network.agents[b].moneySpent[au.auctionID] += au.bBids[end][b]
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

end  # module  Decls
