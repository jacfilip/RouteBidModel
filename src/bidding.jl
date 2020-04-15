
"""
Represents parameters for a two-road transportation system
"""



@with_kw struct BidModelParams
    N_MAX::Int = 8   #number of agents
    ct::Vector{Float64} =  vcat(
       [10 + (rand() - 0.5) * 10 for i in 1:3],
       [10 + (rand() - 0.5) * 10 for i in 4:N_MAX]) ./ 3600 #cost of time $/s
    #real cost of time USDs/s for each agent
    cf::Float64 = 2.1 / 1000 # cost of fuel USD/m
    d::NTuple{2,Float64} = (1000.0, 1000.0)  #road length m
    N::NTuple{2,Int} = (14, 7)        #road max capacity
    v_min::NTuple{2,Float64}  = (1 /3.6, 1 / 3.6)  #minimal velocity m/s
    v_max::NTuple{2,Float64} = (50 / 3.6, 50 / 3.6) #maximal velocity m/s
    r_len::Matrix{Float64} = zeros(Float64, (N_MAX,2))
end

function calculate_nash(s::Simulation)
    N_MAX = s.network.agentIDmax
    ct = [s.network.agents[i].valueOfTime for i in 1:s.network.agentIDmax] #$/s
    cf = s.p.CoF #$/m
    travel_counts = [0, 0]

    roads = [get_road_by_nodes(s.network, s.p.east_bridge_lane...), get_road_by_nodes(s.network, s.p.west_bridge_lane...)]

    empty!(roads[1].agents)
    empty!(roads[2].agents)

    x = Vector{Int}(undef, N_MAX)
    cost_agent = Vector{Float64}(undef, N_MAX) #$/s
    time_agent = Vector{Float64}(undef, N_MAX) #s

    #assign agents to their best choice routes
    for n in 1:N_MAX
        a = s.network.agents[n]
        times = travel_times(s, a) #s
        costs = a.valueOfTime .* times + s.p.CoF .* a.routesDist
        if costs[1] < costs[2]
            travel_counts[1] += 1
            push!(roads[1].agents, n)
            x[n] = 0
            cost_agent[n] = costs[1] #$/s
            time_agent[n] =  times[1] #s
        else
            travel_counts[2] += 1
            push!(roads[2].agents, n)
            x[n] = 1
            cost_agent[n] = costs[2] #$/s
            time_agent[n] = times[2] #s
        end
    end

    #calculate final costs
    for i in 1:N_MAX
        a = s.network.agents[i]
        time_agent[i] = travel_times(s, a)[x[i] + 1]
        cost_agent[i] =  a.valueOfTime * time_agent[i] + s.p.CoF * a.routesDist[x[i] + 1]
    end

    return TravelPattern(x, cost_agent, cost_agent, travel_counts..., time_agent) #, [], []
end

function calculate_optimal_jump(s::Simulation,
    bid_ct=[s.network.agents[i].valueOfTime for i in 1:s.network.agentIDmax])
    N_MAX = s.network.agentIDmax
    N = s.p.bridge_capacity
    cf = s.p.CoF #$/m
    blens = [get_road_by_nodes(s.network, s.p.east_bridge_lane...).rlen, get_road_by_nodes(s.network, s.p.west_bridge_lane...).rlen] #m

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
    #println("Cost: $(objective_value(m))")
    #println("n0=$(value(n0))")
    #println([round(value(x[i])) for i in 1:N_MAX])
    #this cost array here contains all costs [in $] from the objective above
    cost = [trunc(Int, round(value(x[i]))) == 0 ? cf * as[i].routesDist[1] + bid_ct[i] * (blens[1] / ((s.p.b_vmax[1] - s.p.b_vmin[1]) * (1 - value(n0) / N[1]) + s.p.b_vmin[1]) - bts[1] + as[i].routesTime[1]) :
     cf * as[i].routesDist[2] + bid_ct[i] * (blens[2] / ((s.p.b_vmax[2] - s.p.b_vmin[2]) * (1 - value(n1) / N[2]) + s.p.b_vmin[2]) - bts[2] + as[i].routesTime[2]) for i in 1:N_MAX]

    time = [trunc(Int, round(value(x[i]))) == 0 ? (blens[1] / ((s.p.b_vmax[1] - s.p.b_vmin[1]) * (1 - value(n0) / N[1]) + s.p.b_vmin[1]) - bts[1] + as[i].routesTime[1]) :
      (blens[2] / ((s.p.b_vmax[2] - s.p.b_vmin[2]) * (1 - value(n1) / N[2]) + s.p.b_vmin[2]) - bts[2] + as[i].routesTime[2]) for i in 1:N_MAX]

    #(bids, log) = play_nash(bid_params, bid_params.ct, 3)

    TravelPattern([trunc(Int, round(value(x[i]))) for i in 1:N_MAX], cost, cost, Int(round(value(n0))), Int(round(value(n1))),time) #, [], []
end



function travel_times(s::Simulation, a::Agent)
    p = s.p

    r = [get_road_by_nodes(s.network, p.east_bridge_lane...), get_road_by_nodes(s.network, p.west_bridge_lane...)]

    t_mins = [r[i].rlen / r[i].vMax for i in 1:2] #travel times at empty brides [s]

    #current travel times through the brides [s]
    t_cur = [r[i].rlen / lin_k_f_model(length(r[i].agents), p.bridge_capacity[i], r[i].vMax) for i in 1:2]

    return a.routesTime .+ t_cur .- t_mins #s
end


"""
Represents result of an optimization run on a set of model parameters
"""
@with_kw struct TravelPattern
    x::Vector{Int} #allocation for each traffic participant
    cost::Vector{Float64} # cost occured by each agent /with regards to bids/
    cost_real::Vector{Float64} = Float64[] #real cost occured by each agent
    n0::Int # number of agents taking the first road
    n1::Int # number of agents taking the second road
    ts::Vector{Float64} = Float64[] # travel times for both roads
    #bids::Vector{Float64} = Float64[]
    #payments::Vector{Float64} = Float64[]
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
    NashEq(p::BidModelParams)

Finds a time-balanced layout for a two road system described by `p`
"""
function NashEq(p::BidModelParams)
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
    # averages assuming that the agen  ends up within a given location
    # by random - and hence this are the expected values
    ttime = best_ts'*[n0,n1]/(n0+n1)
    tdist = (p.d[1]*n0+p.d[2]*n1)/(n0+n1)
    NashEq(n0=n0,n1=n1,ts=best_ts,ttime=ttime,tdist=tdist)
end



"""
Represent a payment plan along a travel plan
Contains a `NashEq` for efficiency comaparison purposes
"""
@with_kw struct PaymentPlan
    payments::Vector{Float64}
    travel_plan::TravelPattern
    nasheq::Union{NashEq, Nothing}
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
    #println("cf: $(p.cf)")
    #println("p.d: $(p.d)")
    #println("x: $(x)")
    #println("ct: $(bid_ct.*3600)")
    #println("ts: $(ts)")
    c = p.cf .* p.d[x .+ 1] .+ bid_ct .* ts[x .+ 1]
    #println("cost: $(c)")
    c
end

function solve_nash(p::BidModelParams)::TravelPattern
    n1 = 0
    n2 = 0
    x = Vector{Int}()
    t = Vector{Float64}()
    for i in 1:p.N_MAX
        if n1 == p.N[1]
            @assert n2 < p.N[2]
            n2 += 1
            push!(x, 1)
        else
            ts = t_travel(p,[n1, n2])
            if ts[1] < ts[2]
                n1 = n1 +1
                push!(x, 0)
                push!(t, ts[1])
            else
                n2 = n2 +1
                push!(x, 1)
                push!(t, ts[2])
            end
        end
    end

    #for i in 1:p.N_MAX
    #    t[i] = t_travel(p, [n1,n2])[x[i]+1]
    #end
    t = t_travel(p, [n1,n2])
    return TravelPattern(x=x, cost=cost(p, x, t),cost_real=[],n0=n1,n1=n2,ts=t) #,bids=[],payments=[]
end

#TODO: can be removed after calculations:
function calc_nash(p::BidModelParams)::TravelPattern
    n0 = Int(p.N_MAX / 2 + 1)
    n1 = Int(p.N_MAX / 2 - 1)

    x = vcat(Int.(ones(n1)), Int.(zeros(n0)))
    t = Vector{Float64}()

    #for i in 1:p.N_MAX
    #    push!(t, t_travel(p, [n0,n1])[x[i]+1])
    #end
    t = t_travel(p, [n0,n1])

    return TravelPattern(x=x, cost=cost(p, x, t, p.ct),cost_real=[],n0=n0,n1=n1,ts=t) # ,bids=[],payments=[]
end
"""
    solve_travel_jump(p::BidModelParams, bid_ct=p.ct)::TravelPattern

Finds the optimal travel layout for a given set of parameters and bids.
This function should be used for testing purposes only, use `solve_travel`
instead.
"""
function solve_travel_jump(p::BidModelParams, bid_ct=p.ct;print_level=0)::TravelPattern
  params = Dict{Symbol,Any}()
  params[:nl_solver] = with_optimizer(Ipopt.Optimizer, print_level=print_level)
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
Should return identical results to `solve_travel_jump` and is 250x faster.
"""
function solve_travel(p::BidModelParams, bid_ct=p.ct; debugdf::Union{DataFrame,Nothing}=nothing)::TravelPattern
    @assert length(p.N) == 2
    local best_x::Vector{Int}
    local best_cost_x::Vector{Float64}
    local best_ts::Vector{Float64}
    best_cost = Inf
    # sorted indices of bids
    Δ = p.ct./1000 # this ensures that sorting order matches first bid order
    bid_ct_ixs = sortperm(bid_ct+Δ, rev=true)
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
function solve_optimal_payment(p::BidModelParams, bid_ct::Vector{Float64}=p.ct; nasheq = NashEq(p))::PaymentPlan
    travp = solve_travel(p, bid_ct)
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
    #redistribute_c = extra_money/n_get
    #for i in 1:length(pays)
    #    if Δc[i] < 0
    #        pays[i] -= redistribute_c
    #    end
    #end

    #let's distribute  Δ among the ALL

    pays .-= (extra_money/p.N_MAX)

    #@assert abs(sum(pays)) <= 100*eps(mean(p.ct)*10)
    return PaymentPlan(payments=pays, travel_plan=travp, nasheq=nasheq )
end

function solve_optimal_payment(ne_costs::Vector{Float64}, opt_costs::Vector{Float64})
    Δc =   ne_costs .- opt_costs
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
    return pays
end

function solve_middle_payment(p::BidModelParams, bid_ct::Vector{Float64}=p.ct; nasheq = NashEq(p))::PaymentPlan
    travp = solve_travel(p, bid_ct)
    r_fast, r_slow = travp.ts[1] < travp.ts[2] ? (1,2) : (2,1)
    Δt = travp.ts[r_slow] - travp.ts[r_fast]
    price_slow = maximum(bid_ct[travp.x .== r_slow-1])
    price_fast = minimum(bid_ct[travp.x .== r_fast-1])
    price = (price_slow+price_fast)/2
    pays = (travp.x .== r_slow-1)*(-1)*sum(travp.x .== r_fast-1)/(travp.n0+travp.n1)*price/Δt .+
          (travp.x .== r_fast-1)*sum(travp.x .== r_slow-1)/(travp.n0+travp.n1)*price/Δt
    @assert abs(sum(pays)) < 0.01
    return PaymentPlan(payments=pays, travel_plan=travp, nasheq=nasheq )
end


"""
Revaluates bid for a single user who has a certain information about offers
placed by other market participants.
Returns:
NamedTuple{(:bid, :real_cost, :payment),Tuple{Float64,Float64,PaymentPlan}}
"""
function optimizebid(p::BidModelParams, a::Int, bid_ct::AbstractVector{Float64}=p.ct; nasheq=NashEq(p))
    #Δ = (1+rand())/100000 #this ensures that bids are unique across users
    vals = Vector{Float64}(undef, length(bid_ct)+3)
    for i in 1:length(bid_ct)
        if i == a
            vals[i] = bid_ct[i]
        else
            vals[i] = bid_ct[i] #+Δ
        end
    end
    vals[length(bid_ct)+1] = minimum(bid_ct)-0.01 #-Δ
    vals[length(bid_ct)+2] = p.ct[a]
    vals[length(bid_ct)+3] = (maximum(bid_ct)+p.ct[a])/2
    mybid_ct = deepcopy(bid_ct)
    best_cost = Inf
    local best_s1
    best_bid = 0.0
    for bid_val in vals
        mybid_ct[a] = bid_val
        #println("bid_val ",bid_val)
        s1 = solve_optimal_payment(p, mybid_ct;nasheq=nasheq)
        roadix = s1.travel_plan.x[a]+1
        #myreal_ne_cost = p.cf * s1.nasheq.tdist + p.ct[a] * s1.nasheq.ttime
        my_real_cost = p.cf * p.r_len[a, roadix] + p.ct[a] * s1.travel_plan.ts[roadix] + s1.payments[a]
        if my_real_cost < best_cost
            best_cost = my_real_cost
            best_bid = mybid_ct[a]
            best_s1=s1
            #println(best_bid, " ",best_cost)
        end
    end
    (bid=best_bid, real_cost=best_cost, payment=best_s1)
end


function optimizebid_middle(p::BidModelParams, a::Int, bid_ct::AbstractVector{Float64}=p.ct; nasheq=NashEq(p))
    #Δ = (1+rand())/1000 #this ensures that bids are unique across users
    vals =[bid_ct[a], p.ct[a]]
    for i in 1:length(bid_ct)
        if i != a
            push!(vals,bid_ct[i]) # + Δ
        end
    end
    for r in [0.25,0.5,2,4,8,16,32,64]
        push!(vals,bid_ct[a]*r)
    end
    mybid_ct = deepcopy(bid_ct)
    best_cost = Inf
    local best_s1
    best_bid = 0.0
    for bid_val in vals
        mybid_ct[a] = bid_val
        #println("bid_val ",bid_val)
        s1 = solve_middle_payment(p, mybid_ct;nasheq=nasheq)
        roadix = s1.travel_plan.x[a]+1
        #myreal_ne_cost = p.cf * s1.nasheq.tdist + p.ct[a] * s1.nasheq.ttime
        my_real_cost = p.cf * p.r_len[a, roadix] + p.ct[a] * s1.travel_plan.ts[roadix] + s1.payments[a]
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
    play_global_nash(p::BidModelParams, start_bid::Vector{Float64} = p.ct, N_STEPS::Int=200, optimizebidf::Function=optimizebid, nasheq=NashEq(p))

Plays a Nash bidding game assuming that all bids are public.
In each turn a single agent evaluates her bidding situation and chooses
a bid that maximizes her profit.
"""
function play_global_nash(p::BidModelParams, start_bid::Vector{Float64} = p.ct, N_STEPS::Int=200, optimizebidf::Function=optimizebid, nasheqlocal=NashEq(p))
    current_bid = deepcopy(start_bid)
    logagents = DataFrame(f=String[],i=Int[], a=Int[], bid = Float64[], pmnt = Float64[])
    logsteps =  DataFrame(f=String[],i=Int[], n0=Int[], n1=Int[],
                    cost_real_avg = Float64[],
                    cost1=Float64[], cost2=Float64[],
                    costneq1=Float64[], costneq2=Float64[],
                    pmnt1 = Float64[], pmnt2 = Float64[],
                    neq_t=Float64[], t_1=Float64[], t_2=Float64[] )
    costneq = cost(p,nasheqlocal)
    for i in 1:N_STEPS
        a = ((i - 1) % (length(p.ct))) + 1 #rand(1:p.N_MAX)
        res = optimizebidf(p,a,current_bid, nasheq=nasheqlocal)
        tp = res.payment.travel_plan
        current_bid[a] = res.bid
        pmnts=[Float64[],Float64[] ]
        costs=[Float64[],Float64[] ]
        costsneq=[Float64[],Float64[] ]
        for ia in 1:p.N_MAX
            push!(logagents, [string(optimizebidf), i, ia,current_bid[ia],res.payment.payments[ia]])
            push!(pmnts[tp.x[ia]+1], res.payment.payments[ia])
            push!(costs[tp.x[ia]+1], tp.cost_real[ia])
            push!(costsneq[tp.x[ia]+1], costneq[ia])
        end
        push!(logsteps, [string(optimizebidf), i, tp.n0, tp.n1,
              sum(tp.cost_real)/p.N_MAX,
              mean(costs[1]),mean(costs[2]),
              mean(costsneq[1]),mean(costsneq[2]),
              mean(pmnts[1]),mean(pmnts[2]),
              nasheqlocal.ttime, tp.ts[1], tp.ts[2]]
              )
    end
    (bid=current_bid, logagents=logagents, logsteps=logsteps)
end
