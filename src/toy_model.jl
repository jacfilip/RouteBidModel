include("toy_model_functions.jl")

Random.seed!(0);
p = ModelParams()

ne = solve_nash_time(p)

ne_cost = cost(p,ne)
sum(ne_cost)

opt = solve_travel(p)

Random.seed!(0);
bid6 = 6 .+ rand(length(p.ct))./100
res6 = play_nash(p,bid6; N_STEPS=200)
sol6 = solve_travel(p,res6.bid)
sum(sol6.cost)
d6=DataFrame(x=sol6.x,p=p.ct, bid=res6.bid)
display(sort(d6,:p))

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
