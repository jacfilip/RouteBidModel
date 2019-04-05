module Decls

using LightGraphs, SimpleWeightedGraphs
using OpenStreetMapX


export Network
export Simulation
export Agent
export Road
export Intersection
export InitNetwork!
export MakeAction!

agentCntr = 0
agentIDmax = 0

headway = 1.0
avgCarLen = 5.0

mutable struct Road
    length::Real
    agents::Vector{Int}
    bNode::Int
    fNode::Int
    vMax::Real
    capacity::Int
    lanes::Int

    Road(in::Int, out::Int, len::Float64, vel::Float64, lanes::Int = 1) = (
    r = new();
    r.bNode = in;
    r.fNode = out;
    r.lanes = lanes;
    r.length = len;     #ToDo: calculate length from osm
    r.vMax = vel;       #ToDo: retrieve from osm
    r.agents = Vector{Int}();
    r.capacity = floor(r.length / (avgCarLen + headway)) * r.lanes;
    return r;
    )::Road
end

mutable struct Intersection
    nodeID::Int
    inRoads::Vector{Road}
    outRoads::Vector{Road}
    spawnPoint::Bool
    destPoint::Bool

    Intersection(id::Int, spawnPoint = false, destPoint = false) =(
        inter = new();
        inter.nodeID = id;
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
    reducedGraph::SimpleWeightedDiGraph
    myBids::Vector{Tuple{Int, Real}}
    VoT::Real
    CoF::Real
    carLength::Real
    vMax::Real

    Agent(start::Intersection, dest::Intersection, graph::SimpleWeightedDiGraph) = (
        a = new();
        a.atNode = start;
        a.destNode = dest;
        a.reducedGraph = deepcopy(graph);

        global agentCntr += 1;
        global agentIDmax += 1;
        a.id = agentIDmax;
        a.roadPosition = 0.0;
        a.myBids = Vector{Tuple{Int,Real}}();
        a.VoT = 15.0;            #Value of Time $/min ToDo: Draw value
        a.CoF = 0.15e-3;        #Fuel cost $/m #ToDo: Check and draw value
        a.carLength = 3.0;      #m ToDo: Draw value
        a.vMax = 1500.0;         #m/min ToDo: Draw value
        a.bestRoute = Vector{Int}();
        a.atRoad = nothing;
        return  a;
    )::Agent

    Agent() = new();
end

mutable struct Network
    roads::Vector{Road}
    intersections::Vector{Intersection}
    spawns::Vector{Int}
    dests::Vector{Int}
    numRoads::Int
    agents::Vector{Agent}
    graph::SimpleWeightedDiGraph
    Network(g::SimpleWeightedDiGraph) = (
            x = new();
            x.graph = deepcopy(g);
            x.numRoads = 0;
            x.spawns = Vector{Intersection}();
            x.dests = Vector{Intersection}();
            x.agents = Vector{Agent}();
            return x)::Network
    Network() = new();
end

mutable struct Simulation
    network::Network
    timeMax::Real
    timeStep::Real
    timeToNext::Vector{Tuple{Int,Real}}

    timeElapsed::Real
    isRunning::Bool
    Simulation(n::Network, tmax::Real, dt::Real, run::Bool = true) = (
        s = Simulation();
        s.network = n;
        s.timeMax = tmax;
        s.timeStep = dt;
        s.timeToNext = Vector{Tuple{Int,Real}}();
        s.timeElapsed = 0;
        s.isRunning = run;
    return s;)::Simulation
    Simulation() = new();
end

function GetTimeStep(sim::Simulation)::Real
    dt = Vector{Real}()
    for r in sim.network.roads

    end

    return sim.timeStep
end

function InitNetwork!(n::Network)
    global agentIDmax = 0
    global agentCntr = 0
    n.intersections = Vector{Intersection}(undef,n.graph.weights.m)
    for i in 1:n.graph.weights.m
        n.intersections[i] = Intersection(i)
    end

    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                n.numRoads += 1
                println("$(n.numRoads): $(i) -> $(j) = $(n.graph.weights[i,j])")
            end
        end
    end
    n.roads = Vector{Road}(undef, n.numRoads)

    r = 0
    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                r += 1
                n.roads[r] = Road(i, j, n.graph.weights[i, j], 800.0)
                push!(n.intersections[i].outRoads, n.roads[r])
                push!(n.intersections[j].inRoads, n.roads[r])
            end
        end
    end
end

function SetSpawnsAndDests!(n::Network, spawns::Vector{Int}, dests::Vector{Int})
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

function GetRoadByNodes(n::Network, first::Int, second::Int)::Road
    if first > length(n.intersections) || second > length(n.intersections)
        error("Node of of boundries")
    else
        for r in n.roads
            if r.bNode == first && r.fNode == second
                return r
            end
        end
    end
end

function GetIntersectByNode(n::Network, id::Int)::Intersection
    for i in n.intersections
        if i.nodeID == id
            return i
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

function CanFitAtRoad(a::Agent, r::Road)::Bool
    if length(r.agents) < r.capacity
        return true
    else
        return false
    end
end

function GetVelocity(r::Road)
    return max(lin_k_f_model(length(r.agents) * (avgCarLen + headway), r.length, r.vMax), r.vMax)
end

function SpawnAgentAtRandom(n::Network)
    push!(n.agents, Agent(n.intersections[rand(n.spawns)],n.intersections[rand(n.dests)],n.graph))
end

function RemoveFromRoad!(r::Road, a::Agent)
    a.atRoad = nothing
    deleteat!(r.agents, findall(b -> b == a.id, r.agents))
end

function DestroyAgent(a::Agent, n::Network)
    deleteat!(n.agents, findall(b -> b.id == a.id, n.agents))
    global agentCntr -= 1
end

function SetWeights!(a::Agent, n::Network)
    for r in n.roads
        ttime = r.length / r.GetAvgVelocity
        a.reducedGraph.weights[r.bNode, r.fNode] = ttime * a.VoT + r.length * a.CoF
    end
end

function SetShortestPath!(a::Agent)::Real
    if a.atNode != nothing
        s_star = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID)
        dist = s_star.dists[a.atNode.nodeID]
        if dist != Inf
            nextNode = a.atNode.nodeID
            path = s_star.parents
            empty!(a.bestRoute)
            while nextNode != 0
                nextNode = path[nextNode]
                if nextNode != 0
                    push!(a.bestRoute, nextNode)
                end
            end
            return dist
        else
            return Inf
        end
    end
    return nothing
end

function ReachedIntersection(a::Agent, n::Network)
    if a.atNode == a.destNode
        println("Agent $(a.id) destroyed.")
        DestroyAgent(a, n)
    else
        #ToDo: add SetWeights here
        if SetShortestPath!(a) == Inf
            return
        else                    #turn into a new road section
            #ToDo: Calculate alterRoute too
            nextRoad = GetRoadByNodes(n, a.atNode.nodeID, a.bestRoute[1])
            if (CanFitAtRoad(a, nextRoad))
                push!(nextRoad.agents, a.id)
                a.atRoad = nextRoad
                a.roadPosition = 0.0
                a.atNode = nothing
            end
        end
    end
end

function MakeAction!(a::Agent, sim::Simulation)
    if a.atRoad != nothing
        a.roadPosition += GetVelocity(a.atRoad) * GetTimeStep(sim) / 60.0
        a.roadPosition = min(a.roadPosition, a.atRoad.length)
        if a.roadPosition == a.atRoad.length
            a.atNode = GetIntersectByNode(sim.network, a.atRoad.fNode)
            RemoveFromRoad!(a.atRoad, a)
        end
    end

    if a.atNode != nothing
        ReachedIntersection(a, sim.network)
    end
end

function exp_k_f_model(k::Real, k_max::Real, v_max::Real)
    return v_max * exp(- k / k_max)
end

function lin_k_f_model(k::Real, k_max::Real, v_max::Real)
    return v_max * (1.0 - k / k_max)
end

end  # module  Decls
