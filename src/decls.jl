module Decls

using LightGraphs, SimpleWeightedGraphs
using OpenStreetMapX
using Compose
using DataFrames
using Distributions

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

bigNum = 1000000

agentCntr = 0
agentIDmax = 0

headway = 1.0
avgCarLen = 5.0

muteRegistering = true
simLog = Vector{String}()

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
    curVelocity::Real
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
    r.curVelocity = 0.0;
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

    Intersection(id::Int, posX = 0.0, posY = 0.0, spawnPoint = false, destPoint = false) =(
        inter = new();
        inter.nodeID = id;
        inter.posX = posX;
        inter.posY = posY;
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
        a.vMax = 120.0;         #km/h ToDo: Draw value
        a.bestRoute = Vector{Int}();
        a.atRoad = nothing;
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
    Network(g::SimpleWeightedDiGraph, coords::Vector{Tuple{Float64,Float64,Float64}}) = (
            n = new();
            n.graph = deepcopy(g);
            n.numRoads = 0;
            n.spawns = Vector{Intersection}();
            n.dests = Vector{Intersection}();
            n.agents = Vector{Agent}();
            InitNetwork!(n, coords);
            return n)::Network
    Network(g::SimpleWeightedDiGraph, coords::Vector{ENU}) = (
        return Network(g,[(i.east, i.north, i.up) for i in coords]))::Network
    Network(map::MapData) = (return  ConvertToNetwork(map))::Network
end

mutable struct Simulation
    network::Network
    timeMax::Real
    timeStep::Real
    timeStepVar::Vector{Real}
    dt_min::Real
    iter::Int

    simData::DataFrame
    simLog::Vector{String}

    maxAgents::Int
    maxIter::Int

    timeElapsed::Real
    isRunning::Bool
    Simulation(n::Network, tmax::Real; dt::Real = 0, dt_min = 1.0, maxAgents::Int = bigNum, maxIter::Int = bigNum, run::Bool = true) = (
        s = Simulation();
        s.network = n;
        s.iter = 0;
        s.timeMax = tmax;
        s.timeStep = dt;
        s.dt_min = dt_min;
        s.timeStepVar = Vector{Real}();
        s.timeElapsed = 0;
        s.maxAgents = maxAgents;
        s.isRunning = run;
        s.maxIter = maxIter;
        s.simData = DataFrame(iter = Int[], t = Real[], agent = Int[], node1 = Int[], node2 = Int[], roadPos = Real[], posX = Real[], posY = Real[]);
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
    push!(s.timeStepVar, maximum([t_min, s.dt_min]) + 0.001)
    AddRegistry("Time step: $(s.timeStepVar[s.iter]) at iter $(s.iter)", true)
    return s.timeStepVar[end]
end

function InitNetwork!(n::Network, coords::Vector{Tuple{Float64,Float64,Float64}})
    global agentIDmax = 0
    global agentCntr = 0
    n.intersections = Vector{Intersection}(undef,n.graph.weights.m)
    for i in 1:n.graph.weights.m
        n.intersections[i] = Intersection(i, coords[i][1], coords[i][2])
    end

    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                n.numRoads += 1
                AddRegistry("$(n.numRoads): $(i) -> $(j) = $(n.graph.weights[i,j])", true)
            end
        end
    end
    n.roads = Vector{Road}(undef, n.numRoads)

    r = 0
    for i in 1:n.graph.weights.m
        for j in 1:n.graph.weights.n
            if n.graph.weights[i, j] != 0
                r += 1
                n.roads[r] = Road(i, j, n.graph.weights[i, j], 60.0)
                push!(n.intersections[i].outRoads, n.roads[r])
                push!(n.intersections[j].inRoads, n.roads[r])
            end
        end
    end
    AddRegistry("Network has been successfully initialized.")
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

    return Network(g, [m.nodes[m.n[i]] for i in 1:length(m.n)])
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

function GetIntersectionCoords(n::Network)::Vector{Tuple{Tuple{Real,Real},Tuple{Real,Real}}}
    tups = (i -> (i.posX, i.posY)).(n.intersections)

    return pts = [((tups[r.bNode][1], tups[r.bNode][2]), (tups[r.fNode][1], tups[r.fNode][2])) for r in n.roads]
end

function GetIntersectionCoords2(n::Network)::DataFrame
    df = DataFrame(first_X = Real[], second_X = Real[], first_Y = Real[],  second_Y = Real[])
    for p in GetIntersectionCoords(n)
        push!(df, Dict( :first_X => p[1][1], :first_Y => p[1][2], :second_X => p[2][1], :second_Y => p[2][2]))
    end
    return df
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
    if length(r.agents) < r.capacity
        return true
    else
        return false
    end
end

function SetVelocityMpS!(r::Road)
    r.curVelocity = min(lin_k_f_model(length(r.agents) * (avgCarLen + headway), r.length, r.vMax), r.vMax) / 3.6
end

function SpawnAgents(n::Network, dt::Real, t::Real)
    λ = 5.0    #avg number of vehicles per second that appear
    k = rand(Distributions.Poisson(λ))
    k = maximum([k, 0])
    for i in 1:k
        SpawnAgentAtRandom(n)
    end
end

function SpawnAgentAtRandom(n::Network)
    push!(n.agents, Agent(n.intersections[rand(n.spawns)],n.intersections[rand(n.dests)],n.graph))
    AddRegistry("Agent #$(agentIDmax) has been created.")
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
        ttime = r.length / r.curVelocity
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

function GetAgentLocation(a::Agent, n::Network)::Union{Tuple{Int,Int,Real}, Nothing}
    if(a.atNode != nothing)
        return a.atNode.nodeID, a.atNode.nodeID, 0.
    elseif a.atRoad != nothing
        return a.atRoad.bNode, a.atRoad.fNode, a.roadPosition
    else
        throw(Exception("Unknown position of agent $(a.id)"))
    end
end

function ReachedIntersection(a::Agent, n::Network)
    if a.atNode == a.destNode
        AddRegistry("Agent $(a.id) destroyed.", true)
        DestroyAgent(a, n)
    else
        AddRegistry("Agent $(a.id) reached intersection $(a.atNode.nodeID)", true)
        SetWeights!(a, n)
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
    dt = GetTimeStep(sim)
    if a.atRoad != nothing
        a.roadPosition += a.atRoad.curVelocity * dt
        a.roadPosition = min(a.roadPosition, a.atRoad.length)
        AddRegistry("Agent $(a.id) has travelled $(GetAgentLocation(a, sim.network)[3]) of $(a.atRoad.length) m from $(GetAgentLocation(a, sim.network)[1]) to $(GetAgentLocation(a, sim.network)[2]) at speed: $(a.atRoad.curVelocity)*3.6) km/h", true)
        if a.roadPosition == a.atRoad.length
            a.atNode = GetIntersectByNode(sim.network, a.atRoad.fNode)
            RemoveFromRoad!(a.atRoad, a)
        end
    end
    if a.atNode != nothing
        ReachedIntersection(a, sim.network)
    end
    DumpInfo(a, sim)
end

function DumpInfo(a::Agent, s::Simulation)
    loc = GetAgentLocation(a, s.network)
    if loc == nothing
        return
    else
        (bx, by) = (s.network.intersections[loc[1]].posX, s.network.intersections[loc[1]].posY)
        (fx, fy) = (s.network.intersections[loc[2]].posX, s.network.intersections[loc[2]].posY)
        progress =  a.atRoad == nothing ? 0 : loc[3] / a.atRoad.length
        (x, y) = (bx + progress * (fx - bx), by + progress * (fy - by))

        push!(s.simData, Dict(  :iter => s.iter,
                                :t => s.timeElapsed,
                                :agent => a.id,
                                :node1 => loc[1],
                                :node2 => loc[2],
                                :roadPos => loc[3],
                                :posX => x,
                                :posY => y
                                ))
    end
end

function RunSim(s::Simulation)::Bool
    if !s.isRunning
        return false
    end

    while s.timeElapsed < s.timeMax
        s.iter += 1
        AddRegistry("Iter #$(s.iter), time: $(s.timeElapsed), no. of agents: $(agentCntr), agents in total: $agentIDmax", true)

        for r in s.network.roads
            SetVelocityMpS!(r)
        end

        SpawnAgents(s.network, length(s.timeStepVar) < 2 ? 10 : s.timeStepVar[end] - s.timeStepVar[end-1], s.timeElapsed)

        for a in s.network.agents
            MakeAction!(a, s)
        end

        if s.iter > s.maxIter return false end

        s.timeElapsed += SetTimeStep!(s)
    end
    return true
end

function exp_k_f_model(k::Real, k_max::Real, v_max::Real)
    return v_max * exp(- k / k_max)
end

function lin_k_f_model(k::Real, k_max::Real, v_max::Real = 50.0, v_min::Real = 1.0)
    return (v_max - v_min) * (1.0 - k / k_max) + v_min
end

function DistanceENU(p1::ENU, p2::ENU)
    return sqrt((p1.east - p2.east)^2 + (p1.north - p2.north)^2 + (p1.up - p2.up)^2)
end

function EuclideanNorm(p1::Tuple{Real,Real}, p2::Tuple{Real,Real})
    return sqrt((p1[1] - p2[1])^2 + (p1[2] - p2[2])^2)
end

end  # module  Decls
