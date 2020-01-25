
"""
Represents parameters for a two-road transportation system
"""



@with_kw struct BidModelParams
    N_MAX = 30   #number of agents
    ct =  vcat(
       [10 + (rand() - 0.5) * 10 for i in 1:3],
       [10 + (rand() - 0.5) * 10 for i in 4:N_MAX]) #./ 3600
    #real cost of time USDs/s for each agent
    cf = 2.1 / 1000 # cost of fuel USD/m
    d = [2786.0, 3238.0]  #road length m
    N = [20, 20]        #road max capacity
    v_min  = [1 /3.6, 1 / 3.6]  #minimal velocity m/s
    v_max = [60 / 3.6, 50 / 3.6] #maximal velocity m/s
end

function calculate_nash(s::Simulation)
    N_MAX = s.network.agentIDmax
    ct = [s.network.agents[i].valueOfTime for i in 1:s.network.agentIDmax]
    cf = s.p.CoF
    travel_counts = [0, 0]

    roads = [get_road_by_nodes(s.network, s.p.east_bridge_lane...), get_road_by_nodes(s.network, s.p.west_bridge_lane...)]

    empty!(roads[1].agents)
    empty!(roads[2].agents)

    for n in 1:N_MAX
        a = s.network.agents[n]
        times = travel_times(s, a)
        costs = a.valueOfTime .* times + s.p.CoF .* [roads[1].rlen, roads[2].rlen]
        if costs[1] < costs[2]
            travel_counts[1] += 1
            push!(roads[1].agents, n)
        else
            travel_counts[2] += 1
            push!(roads[2].agents, n)
        end
    end
    return travel_counts
end

function calculate_optimal_jump(s::Simulation,
    bid_ct=[s.network.agents[i].valueOfTime for i in 1:s.network.agentIDmax])
    N_MAX = s.network.agentIDmax
    N = s.p.bridge_capaciy
    cf = s.p.CoF

    blens = [get_road_by_nodes(s.network, s.p.east_bridge_lane...).rlen, get_road_by_nodes(s.network, s.p.west_bridge_lane...).rlen]
    bts = blens./s.p.b_vmax # times of travel through bridges with max speed
    params = Dict{Symbol,Any}()
    params[:nl_solver] = with_optimizer(Ipopt.Optimizer, print_level=0)
    params[:log_levels] = Symbol[]
    m = Model(with_optimizer(Juniper.Optimizer, params));
    @variable(m, x[1:N_MAX], Bin)
    @variable(m, 0 <= n0 <= N[1], Int)
    @variable(m, 0 <= n1 <= N[2], Int)
    @constraint(m, sum(x) == n1)
    @constraint(m, n0 + n1 == N_MAX)
    as = s.network.agents
    @NLobjective(m, Min, sum(
     ( (1 - x[i])  *  (cf * as[i].routesDist[1] + bid_ct[i] * (blens[1] / ((s.p.b_vmax[1] - s.p.b_vmin[1]) * (1 - n0 / N[1]) + s.p.b_vmin[1]) - bts[1] + as[i].routesTime[1])  )
           + x[i]  *  (cf * as[i].routesDist[2] + bid_ct[i] * (blens[2] / ((s.p.b_vmax[2] - s.p.b_vmin[2]) * (1 - n1 / N[2]) + s.p.b_vmin[2]) - bts[2] + as[i].routesTime[2])  ) )  for i in 1:N_MAX))
    optimize!(m)
    termination_status(m)
    println("Cost: $(objective_value(m))")
    println("n0=$(value(n0))")
    println([round(value(x[i])) for i in 1:N_MAX])
    #this cost array here contains all costs from the objective above
    cost = [trunc(Int, round(value(x[i]))) == 0 ? cf * as[i].routesDist[1] + bid_ct[i] * (blens[1] / ((s.p.b_vmax[1] - s.p.b_vmin[1]) * (1 - value(n0) / N[1]) + s.p.b_vmin[1]) - bts[1] + as[i].routesTime[1]) :
     cf * as[i].routesDist[2] + bid_ct[i] * (blens[2] / ((s.p.b_vmax[2] - s.p.b_vmin[2]) * (1 - value(n1) / N[2]) + s.p.b_vmin[2]) - bts[2] + as[i].routesTime[2]) for i in 1:N_MAX]
    TravelPattern([trunc(Int, round(value(x[i]))) for i in 1:N_MAX], cost, Int(round(value(n0))), Int(round(value(n1))),[])

end



function travel_times(s::Simulation, a::Agent)
    p = s.p

    #dists = s.network.mapData.e[p.east_bridge_lane...], s.network.mapData.e[p.west_bridge_lane...]
    r = [get_road_by_nodes(s.network, p.east_bridge_lane...), get_road_by_nodes(s.network, p.west_bridge_lane...)]

    t_mins = [r[i].rlen / r[i].vMax for i in 1:2]
    #TODO Add capacity calculations
    t_cur = [r[i].rlen / lin_k_f_model(length(r[i].agents), r[i].cap, r[i].vMax) for i in 1:2]

    return a.routesTime .+ t_cur .- t_mins
end


"""
Represents result of an optimization run on a set of model parameters
"""
@with_kw struct TravelPattern
    x::Vector{Int} #allocation for each traffic participant
    cost::Vector{Float64} # cost occured by each agent /with regards to bids/
    cost_real = Float64[] #real cost occured by each agent
    n0::Int # number of agents taking the first road
    n1::Int # number of agents taking the second road
    ts = Float64[] # travel times for both roads
end

"""
Result of a Nash Equilibrum where agents take roads in a pattern that
travel time on both roads tends to have equal values
"""
@with_kw struct NashEq
    n0::Int
    n1::Int
    ts::Vector{Float64}
    ttime::Float64
    tdist::Float64
end

"""
Represent a payment plan along a travel plan
Contains a `NashEq` for efficiency comaparison purposes
"""
@with_kw struct PaymentPlan
    payments::Vector{Float64}
    travel_plan::TravelPattern
    nasheq::NashEq
end

"""
    t_travel(p::BidModelParams, nn::AbstractVector{Int})

Travel time for a number of agents occupying each road
"""
@inline function t_travel(p::BidModelParams, nn::AbstractVector{Int})
    @assert all(nn .<= p.N) "The number of travelling agents $nn exceeded the capacity $(p.N)"
    v = (p.v_max .- p.v_min) .* (1 .- (nn ./ p.N) ) .+ p.v_min  #velocity
    p.d ./ v
end

"""
    cost(p::BidModelParams, ne::NashEq, bi_ct=p.ct)

The total cost of travel for a Nash-equilibrum `ne` for a given declared
bid of agents `bid_ct`
"""
@inline function cost(p::BidModelParams, ne::NashEq, bid_ct::AbstractVector{Float64}=p.ct)
    [p.cf * ne.tdist + bid_ct[a] * ne.ttime for a in 1:p.N_MAX]
end

"""
    cost(p::BidModelParams, x::Vector{Int}, ts::Vector{Float64}, bid_ct::AbstractVector{Float64}=p.ct)

The total cost of travel
"""
@inline function cost(p::BidModelParams, x::Vector{Int}, ts::Vector{Float64}, bid_ct::AbstractVector{Float64}=p.ct)
    p.cf .* p.d[x .+ 1] .+ bid_ct .* ts[x .+ 1]
end

"""
    solve_nash_time(p::BidModelParams)

Finds a time-balanced layout for a two road system described by `p`
"""
function solve_nash_time(p::BidModelParams)::NashEq
    @assert length(p.N) == 2
    # assume all people travel the second road
    local best_ts = [Inf, 0]
    local best_ns
    for i=(p.N_MAX-p.N[2]):p.N[1]
        ns = [i, p.N_MAX-i]
        ts = t_travel(p,ns)
        if  abs(ts[2]-ts[1]) < abs(best_ts[1]-best_ts[2])
            best_ns = ns
            best_ts = ts
        end
    end
    n0 = best_ns[1]
    n1 = best_ns[2]
    # we replace actual travel times and travel distances  with the weighted
    # avarages assuming that the agen  ends up within a given location
    # by random - and hence this are the expected values
    ttime = best_ts'*[n0,n1]/(n0+n1)
    tdist = p.d'*[n0,n1]/(n0+n1)
    NashEq(n0=n0,n1=n1,ts=best_ts,ttime=ttime,tdist=tdist)
end

"""
    solve_travel_jump(p::BidModelParams, bid_ct=p.ct)::TravelPattern

Finds the optimal travel layout for a given set of parameters and bids.
This function should be used for testing purposes only, use `solve_travel`
instead.
"""
function solve_travel_jump(p::BidModelParams, bid_ct=p.ct)::TravelPattern
  params = Dict{Symbol,Any}()
  params[:nl_solver] = with_optimizer(Ipopt.Optimizer, print_level=0)
  params[:log_levels] = Symbol[]
  m = Model(with_optimizer(Juniper.Optimizer, params));
  @variable(m, x[1:p.N_MAX], Bin)
  @variable(m, 0 <= n0 <= p.N[1], Int)
  @variable(m, 0 <= n1 <= p.N[2], Int)
  @constraint(m, sum(x) == n1)
  @constraint(m, n0 + n1 == p.N_MAX)
  @NLobjective(m, Min, sum(
    (p.cf * p.d[1] + bid_ct[i] * p.d[1] / ((p.v_max[1] - p.v_min[1]) * (1 - n0 / p.N[1]) + p.v_min[1])) * (1 - x[i])
  + (p.cf * p.d[2] + bid_ct[i] * p.d[2] / ((p.v_max[2] - p.v_min[2]) * (1 - n1 / p.N[2]) + p.v_min[2])) * x[i]
  for i in 1:p.N_MAX))
  optimize!(m)
  termination_status(m)
  cost = [trunc(Int, round(value(x[i]))) == 0 ? (p.cf * p.d[1] + bid_ct[i] * p.d[1] / ((p.v_max[1] - p.v_min[1]) * (1 - value(n0) / p.N[1]) + p.v_min[1])) :
   (p.cf * p.d[2] + bid_ct[i] * p.d[2] / ((p.v_max[2] - p.v_min[2]) * (1 - value(n1) / p.N[2]) + p.v_min[2])) for i in 1:p.N_MAX]
  TravelPattern(x=round.(Int,value.(x)), cost=cost, n0=round(Int,value(n0)), n1=round(Int,value(n1)),ts=[])
end

"""
    solve_travel(p::BidModelParams, bid_ct=p.ct)::TravelPattern

Finds the optimal travel layout for a given set of parameters and bids.
Should return identical results to `solve_travel` and is 250x faster.
"""
function solve_travel(p::BidModelParams, bid_ct=p.ct; debugdf::Union{DataFrame,Nothing}=nothing)::TravelPattern
    @assert length(p.N) == 2
    local best_x::Vector{Int}
    local best_cost_x::Vector{Float64}
    local best_ts::Vector{Float64}
    best_cost = Inf
    # sorted indices of bids
    bid_ct_ixs = sortperm(bid_ct, rev=true)
    #this is used to reverse mapping to sorted values
    rev_bid_ct_ixs = last.(sort!(bid_ct_ixs .=> 1:length(bid_ct_ixs)))
    for i in 1:p.N_MAX
        # agents taking: first road, second road
        z = vcat(zeros(Int, i), ones(Int, p.N_MAX-i))
        for flip in [false, true]
            ns = [i, p.N_MAX-i]
            flip && reverse!(ns)
            !(ns[1] in (p.N_MAX-p.N[2]):p.N[1]) && continue
            x_temp = flip ? (1 .- z) : z
            #println(flip," ",x_temp)
            x = x_temp[rev_bid_ct_ixs]
            #println(flip," ",x)
            @assert length(x) - sum(x) == ns[1]
            @assert sum(x) == ns[2]
            ts = t_travel(p, ns)
            travel_cost = cost(p, x , ts, bid_ct )
            total_travel_cost = sum(travel_cost)
            if debugdf != nothing
                append!(debugdf,Dict( :i=>i, :flip=>flip, :total_travel_cost => total_travel_cost, :x => [x], :x_temp=> [x_temp] ) )
            end
            if total_travel_cost < best_cost
                best_cost = total_travel_cost
                best_cost_x = travel_cost
                best_ts = ts
                best_x = x
            end
        end
    end
    n₁ = sum(best_x)
    TravelPattern(x=best_x, cost=best_cost_x, cost_real=cost(p, best_x , best_ts, p.ct ), n0=p.N_MAX-n₁, n1=n₁, ts=best_ts)
end

"""
This is used to test the heuristics implemented in `solve_travel`
"""
function test_solve_travel()
    Random.seed!(0)
    for i in 1:50
        p = ModelParams()
        @time res1 = solve_travel_jump(p)
        @time res2 = solve_travel(p)
        @assert res1.n0  == res2.n0
        @assert res1.n1 == res2.n1
        @assert res1.x == res2.x
        @assert res1.cost ≈ res2.cost
    end
end



"""
    solve_optimal_payment(p::BidModelParams, bid_ct::Vector{Float64}=p.ct)

Allocates a set of payments for a given set of bids.
"""
function solve_optimal_payment(p::BidModelParams, bid_ct::Vector{Float64}=p.ct)::PaymentPlan
    travp = solve_travel(p, bid_ct)
    nasheq = solve_nash_time(p)
    Δc =   cost(p,nasheq, bid_ct) .- travp.cost
    #display(DataFrame(Δc=Δc))
    extra_money = sum(Δc)
    # this can be not always true due to that in NE there is not control
    # how agent choose their paths compared to NE
    # @assert extra_money >= 0
    extra_money = max(0.0,extra_money)
    n_pay = sum(Δc .> 0)
    n_get =  sum(Δc .< 0)
    pays = deepcopy(Δc)
    #let's distribute  Δ among the poor
    redistribute_c = extra_money/n_get
    for i in 1:length(pays)
        if Δc[i] < 0
            pays[i] -= redistribute_c
        end
    end
    #@assert abs(sum(pays)) <= 100*eps(mean(p.ct)*10)
    return PaymentPlan(payments=pays, travel_plan=travp, nasheq=nasheq )
end

"""
Revaluates bid for a single user who has a certain information about offers
placed by other market participants.
Returns:
NamedTuple{(:bid, :real_cost, :payment),Tuple{Float64,Float64,PaymentPlan}}
"""
function optimizebid(p::BidModelParams, a::Int, bid_ct::AbstractVector{Float64}=p.ct)
    Δ = (1+rand())/100000 #this ensures that bids are unique across users
    vals = Vector{Float64}(undef, length(bid_ct)+1)
    for i in 1:length(bid_ct)
        if i == a
            vals[i] = bid_ct[i]
        else
            vals[i] = bid_ct[i]+Δ
        end
    end
    vals[length(bid_ct)+1] = minimum(bid_ct)-Δ
    mybid_ct = deepcopy(bid_ct)
    best_cost = Inf
    local best_s1
    best_bid = 0.0
    for bid_val in vals
        mybid_ct[a] = bid_val
        #println("bid_val ",bid_val)
        s1 = solve_optimal_payment(p, mybid_ct)
        roadix = s1.travel_plan.x[a]+1
        #myreal_ne_cost = p.cf * s1.nasheq.tdist + p.ct[a] * s1.nasheq.ttime
        my_real_cost = p.cf * p.d[roadix] + p.ct[a] * s1.travel_plan.ts[roadix] + s1.payments[a]
        if my_real_cost < best_cost
            best_cost = my_real_cost
            best_bid = mybid_ct[a]
            best_s1=s1
            #println(best_bid, " ",best_cost)
        end
    end
    (bid=best_bid, real_cost=best_cost, payment=best_s1)
end


"""
    play_nash(p::BidModelParams, start_bid = p.ct; N_STEPS=100)

Plays a Nash bidding game assuming that all bids are public.
In each turn a single agent evaluates her bidding situation and chooses
a bid that maximizes her profit.
"""
function play_nash(p::BidModelParams, start_bid = p.ct; N_STEPS=100)
    current_bid = deepcopy(start_bid)
    log = DataFrame(i=Int[], a=Int[], bid = Float64[])
    for i in 1:N_STEPS
        a = rand(1:p.N_MAX)
        res = optimizebid(p,a,current_bid)
        current_bid[a] = res.bid
        push!(log, [i, a, res.bid])
    end
    (bid=current_bid, log=log)
end
