module Obj

using LightGraphs, SimpleWeightedGraphs
using OpenStreetMapX

agentCntr = 0
agentIDmax = 0

mutable struct Road
    length::Real
    agents::Vector{Int}
    bNode::Int
    fNode::Int
    vMax::Real

    Road(in::Int, out::Int, len::Float64, vel::Float64) = (
    s = new();
    s.bNode = in;
    s.fNode = out;
    s.length = len;     #ToDo: calculate length from osm
    s.vMax = vel;       #ToDo: retrieve from osm
    s.agents = Vector{Int}();
    return s;
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
    road::Road
    roadPosition::Real
    atNode::Intersection
    destNode::Intersection
    bestRoute::Vector{Int}
    alterRoute::Vector{Int}
    reducedGraph::SimpleWeightedDiGraph
    myBids::Vector{Tuple{Int, Real}}
    VoT::Real
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
        a.VoT = 15.0;            #ToDo: Draw value
        a.carLength = 5.0;      #ToDo: Draw value
        a.vMax = 100.0;         #ToDo: Draw value
        a.bestRoute = Vector{Int}();
        #ToDo: Calculate shortest path
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

    timeElapsed::Real
    isRunning::Bool
end

function InitNetwork!(n::Network)
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
                n.roads[r] = Road(i, j, n.graph.weights[i, j], 50.0)
                push!(n.intersections[i].outRoads, n.roads[r])
                push!(n.intersections[j].inRoads, n.roads[r])
            end
        end
    end
end

function SetSpawnsAndDests(n::Network, spawns::Vector{Int}, dests::Vector{Int})
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
        if i.id == id
            return i
        end
    end
    return nothing
end

function GetAgentByID(n::Network, id::Int)::Agent
    for i in n.agents
        if i.id == id
            return i
        end
    end
    return nothing
end

function SpawnAgentAtRandom(n::Network)
    s = rand(n.spawns)
    d = rand(n.dests)
    push!(n.agents, Agent(n.intersections[s],n.intersections[d],n.graph))
end

function SetWeights(a::Agent, n::Network)
    for r in n.roads
        agentCount = length(r.agents)
        ttime = r.length / lin_k_f_model(agentCount, r.capcity, r.vMax)
        a.reducedGraph.weights[r.bNode, r.fNode] = ttime * a.VoT
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

function exp_k_f_model(k::Int, k_max::Int, v_max::Real)
    return v_max * exp(- k / k_max)
end

function lin_k_f_model(k::Int, k_max::Int, v_max::Real)
    return v_max * (1.0 - k / k_max)
end

end  # module  Obj
