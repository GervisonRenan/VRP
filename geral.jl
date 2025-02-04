using Test

struct VRP_instances
    n::Int64
    m::Int64
    Q::Int64
    coords::Vector{Tuple{Float64, Float64}}
    d::Vector{Int64}
    c::Matrix{Float64}
end

function create_test_instance()
    return VRP_instances(
        5,                            
        2,                            
        15,                          
        [(0.0, 0.0), (1.0, 3.0), (4.0, 4.0), (6.0, 2.0), (5.0, 0.0)],
        [0, 5, 10, 8, 7],             
        [0.0 10.0 15.0 20.0 25.0;     
         10.0 0.0 35.0 25.0 30.0;
         15.0 35.0 0.0 30.0 20.0;
         20.0 25.0 30.0 0.0 15.0;
         25.0 30.0 20.0 15.0 0.0]
    )
end


function open_demands(file::String)
    d1 = Int64[]
    open(file, "r") do io
        in_demand_section = false
        for line in eachline(io)
            if occursin("DEMAND_SECTION", line)
                in_demand_section = true
            elseif occursin("DEPOT_SECTION", line)
                break
            elseif in_demand_section
                parts = split(line)
                if length(parts) >= 2
                    push!(d1, parse(Int, parts[2]))
                else
                    throw(ArgumentError("Invalid"))
                end
            end
        end
    end

    if isempty(d1)
        throw(ArgumentError("Seccion not found"))
    end
    return d1
end

function calculate_costs(coords)
    num_points = length(coords)
    costs = zeros(Float64, num_points, num_points)
    for i in 1:num_points
        for j in 1:num_points
            costs[i, j] = sqrt((coords[i][1] - coords[j][1])^2 + (coords[i][2] - coords[j][2])^2)
        end
    end
    return costs
end

function test_instance(instance, n1, Q1, k1, coords1::Vector{Tuple{Int64, Int64}})
    @test instance.n == n1
    @test instance.Q == Q1
    @test instance.m == m1
    @test instance.coords == coords1
end

function open_archive(file::String)
    coords1 = Vector{Tuple{Int64, Int64}}()
    n1, Q1, m1 = 0, 0, 0

    open(file, "r") do io
        in_coord_section = false
        for line in eachline(io)
            if occursin("DIMENSION", line)
                n1 = parse(Int, split(line)[end])
            elseif occursin("CAPACITY", line)
                Q1 = parse(Int, split(line)[end])
            elseif occursin("NODE_COORD_SECTION", line)
                in_coord_section = true
            elseif occursin("DEMAND_SECTION", line)
                break
            elseif in_coord_section
                coords_parts = split(line)
                if length(coords_parts) >= 3
                    push!(coords1, (parse(Int, coords_parts[2]), parse(Int, coords_parts[3])))
                else
                    throw(ArgumentError("Invalid Format"))
                end
            end
        end
    end

    if n1 == 0 || Q1 == 0 || isempty(coords1)
        throw(ArgumentError("instructions or format are wrong, please correct"))
    end

    d1 = open_demands(file)
    total_demand = sum(d1[2:end])
    m1 = ceil(Int, total_demand / Q1)
    c1 = calculate_costs(coords1)

    return coords1, VRP_instances(n1, m1, Q1, coords1, d1, c1)
end

using GLPK
using JuMP
using Plots

struct Results
    x::Matrix{Float64}
end

function solve_vrp(instance)
    model = Model(GLPK.Optimizer)

    @variable(model, x[1:instance.n, 1:instance.n], Bin)
    @variable(model, u[2:instance.n], Int)

    @objective(model, Min, sum(instance.c[i, j] * x[i, j] for i in 1:instance.n, j in 1:instance.n))

    valid_pairs = [(i, j) for i in 2:instance.n, j in 2:instance.n if i != j]

    @constraints(model, begin
        [j in 2:instance.n], sum(x[i, j] for i in 1:instance.n if i != j) == 1
        [i in 2:instance.n], sum(x[i, j] for j in 1:instance.n if i != j) == 1

        sum(x[1, j] for j in 2:instance.n) == instance.m
        sum(x[i, 1] for i in 2:instance.n) == instance.m
    end)

    for (i, j) in valid_pairs
        @constraint(model, u[j] ≥ u[i] + instance.d[j] - (instance.Q + 1) * (1 - x[i, j]))
    end

    @constraint(model, [i in 2:instance.n], u[i] ≥ instance.d[i])
    @constraint(model, [i in 2:instance.n], u[i] ≤ instance.Q)

    optimize!(model)

    if termination_status(model) == MOI.OPTIMAL
        total_distance = objective_value(model)
        println("Costs: $total_distance")
        return Results(value.(x))
    else
        println("No optimal solution")
        return nothing
    end
end

function show_results(instance, results::Results)
    x = results.x
    routes = []
    total_cost = 0

    visited = falses(instance.n)

    for m in 1:instance.K
        route = []
        current_node = 1
        push!(route, current_node)

        current_load = 0
        while true
            found_next = false
            for j in 2:instance.n 
                if x[current_node, j] >= 0.5 && !visited[j] && (current_load + instance.d[j] <= instance.Q)
                    push!(route, j)
                    visited[j] = true
                    current_load += instance.d[j]
                    current_node = j
                    found_next = true
                    break
                end
            end
            if !found_next
                push!(route, 1)
                break
            end
        end

        push!(routes, route)
    end

    for (i, route) in enumerate(routes)
        println("Route #$i: ", join(route, " "))
    end

    total_cost = sum(instance.c[i, j] * x[i, j] for i in 1:instance.n, j in 1:instance.n)
    println("Cost: $total_cost")

    save_archive(routes, total_cost)
    plot_routes(instance, routes)
end


function plot_routes(instance, routes)
    plot()
    for route in routes
        x_coords = [instance.coords[i][1] for i in route]
        y_coords = [instance.coords[i][2] for i in route]
        scatter!(x_coords, y_coords, label="", marker=:circle)
        plot!(x_coords, y_coords, label="Route", arrow=true)
    end
    scatter!([instance.coords[1][1]], [instance.coords[1][2]], label="Depot", marker=:star, color=:red)
    title!("VRP Solution")
    xlabel!("X")
    ylabel!("Y")
end

function save_archive(routes, total_cost)
    open("Results", "w") do io
        for (i, route) in enumerate(routes)
            println(io, "Route $(i): ", join(route, " "))
        end
        println(io, "Costs: ", total_cost)
    end
end


export open_archive, build_vrp_model, solve_vrp, show_results, create_test_instance

file = "C:/Users/gervi/OneDrive/Área de Trabalho/PIBITI 2024/teste.txt.txt"
coords, instance = open_archive(file)
result = solve_vrp(instance)
show_results = (instance, result)
