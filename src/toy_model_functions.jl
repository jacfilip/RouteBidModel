using JuMP
using LinearAlgebra
using Juniper
using Ipopt
using DelimitedFiles
using Parameters
using Random
using Statistics

using DataFrames


"""
Represents parameters for a two-road transportation system
"""
@with_kw struct ModelParams
    N_MAX = 251   #number of agents
    ct = [0.00749, 0.00640, 0.00837, 0.00689, 0.00563, 0.00751, 0.00791, 0.00732, 0.00746, 0.00655, 0.00646, 0.00565, 0.00509, 0.00716, 0.00660, 0.00664, 0.00729, 0.00763, 0.00809, 0.00612, 0.00638, 0.00708, 0.00632, 0.00736, 0.00592, 0.00720, 0.00702, 0.00728, 0.00626, 0.00714, 0.00617, 0.00467, 0.00812, 0.00625, 0.00529, 0.00582, 0.00643, 0.00679, 0.00629, 0.00700, 0.00650, 0.00838, 0.00720, 0.00768, 0.00725, 0.00693, 0.00744, 0.00481, 0.00666, 0.00556, 0.00705, 0.00728, 0.00672, 0.00656, 0.00724, 0.00784, 0.00576, 0.00686, 0.00788, 0.00586, 0.00534, 0.00773, 0.00637, 0.00692, 0.00635, 0.00629, 0.00741, 0.00654, 0.00690, 0.00637, 0.00774, 0.00718, 0.00713, 0.00594, 0.00637, 0.00585, 0.00667, 0.00706, 0.00654, 0.00729, 0.00613, 0.00757, 0.00653, 0.00656, 0.00646, 0.00663, 0.00615, 0.00670, 0.00646, 0.00698, 0.00747, 0.00645, 0.00623, 0.00592, 0.00635, 0.00646, 0.00833, 0.00526, 0.00787, 0.00864, 0.00553, 0.00768, 0.00613, 0.00788, 0.00459, 0.00737, 0.00797, 0.00562, 0.00649, 0.00621, 0.00594, 0.00624, 0.00699, 0.00867, 0.00684, 0.00723, 0.00783, 0.00649, 0.00790, 0.00621, 0.00608, 0.00706, 0.00692, 0.00711, 0.00637, 0.00768, 0.00660, 0.00505, 0.00643, 0.00732, 0.00628, 0.00686, 0.00820, 0.00705, 0.00610, 0.00491, 0.00567, 0.00673, 0.00688, 0.00724, 0.00674, 0.00742, 0.00690, 0.00720, 0.00639, 0.00686, 0.00781, 0.00832, 0.00730, 0.00851, 0.00712, 0.00667, 0.00722, 0.00503, 0.00821, 0.00694, 0.00670, 0.00646, 0.00689, 0.00638, 0.00644, 0.00534, 0.00631, 0.00622, 0.00718, 0.00648, 0.00732, 0.00562, 0.00689, 0.00726, 0.00831, 0.00581, 0.00525, 0.00917, 0.00579, 0.00860, 0.00656, 0.00632, 0.00695, 0.00735, 0.00562, 0.00672, 0.00829, 0.00566, 0.00834, 0.00639, 0.00876, 0.00702, 0.00687, 0.00831, 0.00674, 0.00593, 0.00654, 0.00662, 0.00577, 0.00588, 0.00521, 0.00732, 0.00716, 0.00748, 0.00782, 0.00742, 0.00743, 0.00849, 0.00607, 0.00694, 0.00672, 0.00567, 0.00802, 0.00791, 0.00655, 0.00686, 0.00800, 0.00539, 0.00688, 0.00639, 0.00710, 0.00628, 0.00620, 0.00670, 0.00649, 0.00684, 0.00723, 0.00571, 0.00560, 0.00850, 0.00617, 0.00681, 0.00618, 0.00758, 0.00620, 0.00653, 0.00501, 0.00750, 0.00650, 0.00719, 0.00710, 0.00736, 0.00668, 0.00659, 0.00656, 0.00934, 0.00588, 0.00796, 0.00746, 0.00743, 0.00641, 0.00668, 0.00656, 0.00583, 0.00711, ]
    #vcat(
     #  [10 + (rand() - 0.5) * 10 for i in 1:3],
     #  [10 + (rand() - 0.5) * 10 for i in 4:N_MAX]) #./ 3600
       #real cost of time USDs/s for each agent
    cf = 2.1 / 1000 # cost of fuel USD/m
    d = [2786.0, 3238.0]  #road length m
    N = [250, 250]        #road max capacity
    v_min  = [1 /3.6, 1 / 3.6]  #minimal velocity m/s
    v_max = [60 / 3.6, 50 / 3.6] #maximal velocity m/s
end


"""
Represents result of an optimization run on a set of model parameters
"""
@with_kw struct TravelPattern
    x::Vector{Int} #allocation for each traffic participant
    cost::Vector{Float64} # cost occured by each agent
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
    t_travel(p::ModelParams, nn::AbstractVector{Int})

Travel time for a number of agents occupying each road
"""
@inline function t_travel(p::ModelParams, nn::AbstractVector{Int})
    v = (p.v_max .- p.v_min) .* (1 .- (nn ./ p.N) ) .+ p.v_min  #velocity
    p.d ./ v
end

"""
    cost(p::ModelParams, ne::NashEq, bi_ct=p.ct)

The total cost of travel for a Nash-equilibrum `ne` for a given declared
bid of agents `bid_ct`
"""
@inline function cost(p::ModelParams, ne::NashEq, bid_ct::AbstractVector{Float64}=p.ct)
    [p.cf * ne.tdist + bid_ct[a] * ne.ttime for a in 1:p.N_MAX]
end

"""
    cost(p::ModelParams, x::Vector{Int}, ts::Vector{Float64}, bid_ct::AbstractVector{Float64}=p.ct)

The total cost of travel
"""
@inline function cost(p::ModelParams, x::Vector{Int}, ts::Vector{Float64}, bid_ct::AbstractVector{Float64}=p.ct)
    p.cf .* p.d[x .+ 1] .+ bid_ct .* ts[x .+ 1]
end

"""
    solve_nash_time(p::ModelParams)

Finds a time-balanced layout for a two road system described by `p`
"""
function solve_nash_time(p::ModelParams)::NashEq
    @assert length(p.N) == 2
    # assume all people travel the second road
    local best_ts = [Inf, 0]
    local best_ns
    for i=0:p.N_MAX
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
    solve_travel_jump(p::ModelParams, bid_ct=p.ct)::TravelPattern

Finds the optimal travel layout for a given set of parameters and bids.
This function should be used for testing purposes only, use `solve_travel`
instead.
"""
function solve_travel_jump(p::ModelParams, bid_ct=p.ct)::TravelPattern
  optimizer = Juniper.Optimizer
  params = Dict{Symbol,Any}()
  params[:nl_solver] = with_optimizer(Ipopt.Optimizer, print_level=0)
  m = Model(with_optimizer(optimizer, params));
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
  println("Cost: $(objective_value(m))")
  println("n0=$(value(n0))")
  println([round(value(x[i])) for i in 1:p.N_MAX])
  cost = [trunc(Int, round(value(x[i]))) == 0 ? (p.cf * p.d[1] + bid_ct[i] * p.d[1] / ((p.v_max[1] - p.v_min[1]) * (1 - value(n0) / p.N[1]) + p.v_min[1])) :
   (p.cf * p.d[2] + bid_ct[i] * p.d[2] / ((p.v_max[2] - p.v_min[2]) * (1 - value(n1) / p.N[2]) + p.v_min[2])) for i in 1:p.N_MAX]
  TravelPattern([trunc(Int, round(value(x[i]))) for i in 1:p.N_MAX], cost, Int(round(value(n0))), Int(round(value(n1))),[])
end

"""
    solve_travel(p::ModelParams, bid_ct=p.ct)::TravelPattern

Finds the optimal travel layout for a given set of parameters and bids.
Should return identical results to `solve_travel` and is 250x faster.
"""
function solve_travel(p::ModelParams, bid_ct=p.ct; debugdf::Union{DataFrame,Nothing}=nothing)::TravelPattern
    @assert length(p.N) == 2
    local best_x::Vector{Int}
    local best_cost_x::Vector{Float64}
    local best_ts::Vector{Float64}
    best_cost = Inf
    # sorted indices of bids
    bid_ct_ixs = sortperm(bid_ct, rev=true)
    #this is used to reverse mapping to sorted values
    rev_bid_ct_ixs = last.(sort!(bid_ct_ixs .=> 1:length(bid_ct_ixs)))
    for i=0:p.N_MAX
        # agents taking: first road, second road
        z = vcat(zeros(Int, i), ones(Int, p.N_MAX-i))
        for flip in [false, true]
            x_temp = flip ? (1 .- z) : z
            x = x_temp[rev_bid_ct_ixs]
            ns = [i, p.N_MAX-i]
            flip && reverse!(ns)
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
    TravelPattern(best_x, best_cost_x, p.N_MAX-n₁, n₁,best_ts)
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
    solve_optimal_payment(p::ModelParams, bid_ct::Vector{Float64}=p.ct)

Allocates a set of payments for a given set of bids.
"""
function solve_optimal_payment(p::ModelParams, bid_ct::Vector{Float64}=p.ct)::PaymentPlan
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
function optimizebid(p::ModelParams, a::Int, bid_ct::AbstractVector{Float64}=p.ct)
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
    play_nash(p::ModelParams, start_bid = p.ct; N_STEPS=100)

Plays a Nash bidding game assuming that all bids are public.
In each turn a single agent evaluates her bidding situation and chooses
a bid that maximizes her profit.
"""
function play_nash(p::ModelParams, start_bid = p.ct; N_STEPS=100)
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
