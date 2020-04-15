using Revise
using Pkg
Pkg.activate(".")
using RouteBidModel
using DataFrames
using Random
using Distributions
using CSV
using Combinatorics
using DelimitedFiles

Random.seed!(0);

N_MAX = 8
p = BidModelParams(N_MAX=N_MAX,N=[20,10],ct=[1.0, 60.0, 80.0, 3.0, 100.0, 150.0, 5.0, 200.0] ./ 3600 )

ne = solve_nash(p)

ne = NashEq(p)

e_cost = sum(cost(p, ne))

opt = solve_travel_jump(p)

perms = collect(permutations([1.0, 3.0, 5.0, 60.0, 80.0, 100.0, 150.0, 200.0] ./ 3600))

costs = Vector{Float64}()
for i in [1, 40320]#length(perms)
    p = BidModelParams(N_MAX=N_MAX,N=[20,10],ct=perms[i])
    ne = calc_nash(p)
    #opt = solve_travel_jump(p)
    push!(costs, sum(ne.cost))
end

p2 = BidModelParams(N_MAX=N_MAX,N=[20,10],ct=[1.0, 3.0, 5.0, 60.0, 80.0, 100.0, 150.0, 200.0] ./ 3600 )
res = calc_nash(p2)
res2 = solve_travel_jump(p2)

sum(res.cost) - sum(res2.cost)
c1 = cost(p2, [1,1,1,0,0,0,0,0], t_travel(p2, [3,5]), perms[1])
c2 = cost(p2, [1,1,1,0,0,0,0,0], t_travel(p2, [3,5]), perms[end])
costs_diff = costs .- sum(opt.cost)
(minimum(costs_diff), mean(costs_diff), maximum(costs_diff))

opt = solve_travel_jump(p2)

ts = t_travel(p2, [5,3])
xs = [1,1,1,0,0,0,0,0]
ds = p2.d
cf = p2.cf

costs2 = Vector{Vector{Float64}}()
for i in 1:length(perms)
    ics = cf .* ds[xs .+ 1] .+ perms[i] .* ts[xs .+ 1]
    push!(costs2, ics)
end

c_opt = sum(opt.cost)
cost_diff = costs2 .- c_opt

(minimum(cost_diff), mean(cost_diff), maximum(cost_diff))

writedlm("./results/mean_ne.csv", cost_diff)

sum(t.cost)

opt = solve_travel(p)
sum(opt.cost)
sum(opt.cost_real)


Random.seed!(0);
bid6 = minimum(p.ct) .+ rand(length(p.ct))./10 .* p.ct
res6 = play_nash(p,bid6; N_STEPS=120)
sol6 = solve_travel(p,res6.bid)
sum(sol6.cost)
sum(sol6.cost_real)

pp = solve_optimal_payment(p,res6.bid)
pays = pp.payments
sum(pays)


sol6_ = solve_travel_jump(p,res6.bid)
sum(sol6_.cost)

d6=DataFrame(x=sol6.x,p=p.ct, bid=res6.bid)
display(sort(d6,:p))

res6


Random.seed!(0);
bid12 = 12 .+ rand(length(p.ct))./100
res12 = play_nash(p,bid12; N_STEPS=200)
sol12 = solve_travel(p,res12.bid)
sum(sol12.cost)
d12=DataFrame(x=sol12.x,p=p.ct, bid=res12.bid)
display(sort(d12,:bid))

Random.seed!(0);
bid_rand = 5 .+ rand(length(p.ct)) .* 10
res_rand = play_nash(p,bid_rand; N_STEPS=200)
sol_rand = solve_travel(p,res_rand.bid)
sum(sol_rand.cost)
d_rand=DataFrame(x=sol_rand.x,p=p.ct, bid=res_rand.bid)
display(sort(d_rand,:p))
