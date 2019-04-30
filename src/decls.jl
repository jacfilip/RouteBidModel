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

using Random
Random.seed!(0);

bigNum = 1000000

agentCntr = 0
agentIDmax = 0

headway = 0.25
avgCarLen = 0.5

muteRegistering = false
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
    BorS::Int64
    destNode::Intersection
    bestRoute::Vector{Int}
    alterRoute::Vector{Int}

    timeEstim::Real
    deployTime::Real
    requiredArrivalTime::Real
    #graphCopy::SimpleWeightedDiGraph
    reducedGraph::SimpleWeightedDiGraph
    #roadCosts::SparseArrays.SparseMatrixCSC{Float64,Int64}
    #roadBids::Vector{Tuple{Road, Real}}
    VoT_base::Real # Base value of time $/min
    VoT_dev::Real  # value of time deviation regards of time remained -
    CoF::Real      # fuel cost $/m
    carLength::Real
    vMax::Real      #maximal car velocity in km/h

    Agent(start::Intersection, dest::Intersection, graph::SimpleWeightedDiGraph) = (
        a = new();
        a.atNode = start;
        a.destNode = dest;
        a.reducedGraph = deepcopy(graph);

        global agentCntr += 1;
        global agentIDmax += 1;
        a.id = agentIDmax;
        a.roadPosition = 0.0;
        a.VoT_base = maximum([rand(Distributions.Normal(24.52/60, 3.0/60)), 0.0]);
        a.VoT_dev = rand() * 8.0;
        a.CoF = 0.15e-3;         #ToDo: Check and draw value
        a.carLength = 3.0;      #m ToDo: Draw value
        a.vMax = 120.0;         #km/h
        a.bestRoute = Vector{Int}();
        a.alterRoute = Vector{Int}();
        a.atRoad = nothing;
        a.BorS = rand(Categorical([0.5,0.5]),1)[1];
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
    undirGraph::SimpleWeightedGraph
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
    initialAgents::Int

    timeElapsed::Real
    isRunning::Bool
    Simulation(n::Network, tmax::Real; dt::Real = 0, dt_min = 1.0, maxAgents::Int = bigNum, maxIter::Int = bigNum, run::Bool = true, initialAgents::Int = 200) = (
        s = Simulation();
        s.network = n;
        s.iter = 0;
        s.timeMax = tmax;
        s.timeStep = dt;
        s.dt_min = dt_min;
        s.timeStepVar = Vector{Real}();
        s.timeElapsed = 0;
        s.maxAgents = maxAgents;
        s.initialAgents = floor(initialAgents / 2);
        s.isRunning = run;
        s.maxIter = maxIter;
        s.simData = DataFrame(iter = Int[], t = Real[], agent = Int[], node1 = Int[], node2 = Int[], roadPos = Real[], posX = Real[], posY = Real[]);
    return s;)::Simulation
    Simulation() = new()
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
                AddRegistry("$(n.numRoads): $(i) -> $(j) = $(n.graph.weights[i,j])")
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

    n.undirGraph = SimpleWeightedGraph()
    add_vertices!(n.undirGraph, length(n.intersections))

    for r in n.roads
        add_edge!(n.undirGraph, r.bNode, r.fNode)
    end

    n.undirGraph.weights .= Inf

    for r in n.roads
        n.undirGraph.weights[r.bNode, r.fNode] = n.graph.weights[r.bNode, r.fNode]
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
    return length(r.agents) < r.capacity #Check theres room on road for agent
end

function SetVelocityMpS!(r::Road) #Set Velocity function
    r.curVelocity = min(lin_k_f_model(length(r.agents) * (avgCarLen + headway), r.length, r.vMax), r.vMax) / 3.6
    #Set the current velocity all cars are going on this road based on length, number of agents on the road (congestion), etc.
end

function SpawnAgents(s::Simulation, dt::Real, t::Real)
    if s.maxAgents == agentCntr
        return
    end

    λ = 5.0    #avg number of vehicles per second that appear
    σ = 0.1    #standard deviation as a share of expected travel time
    ϵ = 0.05   #starting time glut as a share of travel time
    k = minimum([maximum([rand(Distributions.Poisson(λ * dt)), 0]), s.maxAgents - agentCntr])
    for i in 1:k
        SpawnAgentAtRandom(s.network, s.timeElapsed)
        dt = EstimateTime(s.network, s.network.agents[end].atNode.nodeID, s.network.agents[end].destNode.nodeID)
        s.network.agents[end].requiredArrivalTime = rand(Distributions.Normal(s.timeElapsed + (1.0+ϵ) * dt, σ * dt))
    end
end

function SpawnAgentAtRandom(n::Network)
    start = Int64
    dest = Int64
    create_agent = false

    while !create_agent
        start = rand(n.spawns)
        dest = rand(n.dests)
        create_agent = IsRoutePossible(n,start,dest)
    end
    push!(n.agents, Agent(n.intersections[start],n.intersections[dest],n.graph)) #Randomly spawn an agent within the outer region
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
        a.reducedGraph.weights[r.bNode, r.fNode] = r.ttime * GetVoT(a, s.timeElapsed) + r.length * a.CoF
    end
end

function SetShortestPath!(a::Agent)::Real #When would this return INFINITY??
    if a.atNode != nothing #If the agent is at the
        s_star = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID) #Find the shortest path between the node currently at and destination node
        dist = s_star.dists[a.atNode.nodeID] #Distance (?)
        if dist != Inf #If distance isn't infinity
            nextNode = a.atNode.nodeID #Next node equals the ID of the node agent is currently on
            path = s_star.parents
            empty!(a.bestRoute) #Remove all elements in this array
            while nextNode != 0 #While nextNode doesn't equal zero
                nextNode = path[nextNode] #Get next road travelled in shortest path
                if nextNode != 0
                    push!(a.bestRoute, nextNode) #Add road to overall shortest path
                end
            end
            return dist
        else
            return Inf
        end
    end
    return nothing
end

#Estimates how much more time agent needs to reach his destination point
function EstimateTime(n::Network, start::Int, dest::Int)::Real
    if start == dest
        return 0
    end

    pth = dijkstra_shortest_paths(n.graph, dest)

    if(pth.dists[start] == Inf)
        return Inf
#        throw(ErrorException("Cannot find route between $(start) and $(dest)."))
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
    timeGlut = a.requiredArrivalTime - (t + a.timeEstim)

    return a.VoT_base * exp(-a.VoT_dev * timeGlut)
end

function SetAlternativeRoute(a::Agent)::Union{Nothing,Real}
    if a.atNode != nothing

    end
end

# function SetYenShortestPaths!(a::Agent)::Real
#     if a.atNode != nothing
#         yen_state = yen_k_shortest_paths(a.reducedGraph, a.destNode.nodeID, 2)
#         if yen_state.dists[1] != Inf
#             a.bestRoute = yen_state.paths[1]
#
#         end
#     end
# end

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
  n = s.network
  if a.atNode == a.destNode #If the the node the agent is at is their destination node
        AddRegistry("Agent $(a.id) destroyed.") #Add to registry that agent has been destroyed
        DestroyAgent(a, n) #Destroy the agent
    else #Otherwise if the agent hasn't yet arrived at their destination
        AddRegistry("Agent $(a.id) reached intersection $(a.atNode.nodeID)")
        SetWeights!(a, s) #Reset weight on edges for agent network
        if SetShortestPath!(a, n) == Inf
            # println("##########")
            println("Agent $(a.id) has a path of infinity. That's not good news!")
            # println("##########")

            return
        else            #turn into a new road section
            # println("We are deleting the path between node $(a.atNode.nodeID) and $(a.bestRoute[1])")
            SetAlternatePath!(a,a.bestRoute[1],a.atNode.nodeID,a.destNode.nodeID)
            if (length(a.alterRoute) != 0) && (length(a.bestRoute) >= 2)
                GrabBuyersGoingDirection(n,a.atNode,a.bestRoute[1])
                # CommenceBidding()
            end
            # println(a.bestRoute)
            # println(a.alterRoute)

            nextRoad = GetRoadByNodes(n, a.atNode.nodeID, a.bestRoute[1]) #Get the road agent is turning on

            if (CanFitAtRoad(a, nextRoad)) #Check that he fits on the road
                push!(nextRoad.agents, a.id) #Add agents ID onto roads agents array
                a.atRoad = nextRoad #Set agent on new road
                a.roadPosition = 0.0 #Set his position on road to zero
                a.atNode = nothing #Agent no longer at node
            else
                println()
                println("CANT FIT TO ROAD!!!! for agent $(a.id)")
                println("The road length $(nextRoad.length)")
                println("The road capacity $(nextRoad.capacity)")
                println("The number of agents currently on that road $(length(nextRoad.agents))")

            end
        end
    end
end

function MakeAction!(a::Agent, sim::Simulation) #Takes an agent ID and simulation
    dt = sim.timeStep
    # println("#######################")
    # if a.atNode != nothing
    #     println("Agent $(a.id) is at node $(a.atNode.nodeID)")
    # elseif a.atRoad != nothing
    #     println("Agent $(a.id) is on the road between $(a.atRoad.bNode) and $(a.atRoad.fNode). He is $(a.roadPosition) of $(a.atRoad.length) done")
    # end
    # println("Now running the iteration and moving agent")
    if a.atNode != nothing
        println("Agent $(a.id) is at the intersection $(a.atNode.nodeID) so we stopped")
        # println("Agent $(a.id) is about to run the function ReachedIntersection")
        ReachedIntersection(a, sim.network)
    end

    if a.atRoad != nothing
        a.roadPosition += a.atRoad.curVelocity * dt #The velocity of the road agent is currently on multiplied by the timestep
        a.roadPosition = min(a.roadPosition, a.atRoad.length) #Take the minimum value between the position they are on the road and the length of that road
        AddRegistry("Agent $(a.id) has travelled $(GetAgentLocation(a, sim.network)[3]) of $(a.atRoad.length) m from $(GetAgentLocation(a, sim.network)[1]) to $(GetAgentLocation(a, sim.network)[2]) at speed: $(a.atRoad.curVelocity)*3.6) km/h")
        if a.roadPosition == a.atRoad.length #If their road position is at the end of the road
            a.atNode = GetIntersectByNode(sim.network, a.atRoad.fNode) #Set the node I'm currently at
            RemoveFromRoad!(a.atRoad, a) #Remove the agent from the road they were on
        end
    end

    DumpInfo(a, sim) #Dump info into dataframe

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
    global once
    if !s.isRunning
        return false
    end
    while s.timeElapsed < s.timeMax
        s.iter += 1 #Add new number of iteration
        AddRegistry("Beginning Iter #$(s.iter)", true)
        for r in s.network.roads #For each road in the network, set the velocity of the road based on congestion, etc.
            SetVelocityMpS!(r)
        end

        #Spawn some number of agents
        SpawnAgents(s, length(s.timeStepVar) == 0 ? 1.0 : s.timeStepVar[end], s.timeElapsed)

        #For each agent in the network
        for a in s.network.agents
            MakeAction!(a, s) #Do action
        end

        s.timeElapsed += SetTimeStep!(s)

        if s.iter > s.maxIter return false end
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

function IsRoutePossible(n::Network, startNode::Int64, endNode::Int64)
    route = LightGraphs.dijkstra_shortest_paths(n.graph,endNode)
    dist = route.dists[startNode]
    if dist == Inf
        return false
    return true
    end
end

function SetAlternatePath!(a::Agent, delNode::Int, bNode::Int, fNode::Int)
    if a.atNode != nothing
        oVal = a.reducedGraph.weights[bNode,delNode]
        a.reducedGraph.weights[bNode,delNode] = Inf
        s_cross = LightGraphs.dijkstra_shortest_paths(a.reducedGraph, a.destNode.nodeID)
        dist = s_cross.dists[a.atNode.nodeID]
        if dist != Inf
            nextNode = a.atNode.nodeID
            path = s_cross.parents
            empty!(a.alterRoute)
            while nextNode !=0
                nextNode = path[nextNode]
                if nextNode != 0
                    push!(a.alterRoute, nextNode)
                end
            end
            a.reducedGraph.weights[bNode,delNode] = oVal
            return dist
        else
            a.reducedGraph.weights[bNode,delNode] = oVal
            return Inf
        end
    end
    return nothing
end

function GrabBuyersGoingDirection(nw::Network, inter::Intersection, destNode::Int64)
    buyers = Agent[]
    for road in inter.inRoads
        if length(road.agents) != 0
            for a in road.agents
                agent = GetAgentByID(nw, a)
                if length(agent.bestRoute) >=2
                    if agent.bestRoute[2] == destNode
                        push!(buyers,agent)
                    end
                end
            end
        end
    end
    return buyers
end

function CommenceBidding()
    println("BID!")
end


end  # module  Decls
