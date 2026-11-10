% benchmark.pl - Run graph search algorithms on a small-world network and emit JSON.
%
% Usage (from graph-search-network/searchs/):
%   swipl -q -f ../scripts/benchmark.pl -- N K P PAIRS_TERM
%
%   N        number of vertices (e.g. 2500)
%   K        k-nearest-neighbours parameter (e.g. 7)
%   P        rewiring probability (e.g. 0.10)
%   PAIRS_TERM  Prolog list of [Start,Goal] pairs, e.g. [[34,2001],[7,12],...]
%
% Output: a single JSON object on stdout with the raw per-pair runs. The graph
% is generated once (deterministic seed 31 from graph.pl) and timing measures
% only the search call - prolog VM startup and graph creation are excluded.
%
% Search algorithms are loaded from ./ (i.e. graph-search-network/searchs/).

:- initialization(main, main).

:- ensure_loaded('../graph.pl').
:- ensure_loaded('./bfs.pl').
:- ensure_loaded('./dfs.pl').
:- ensure_loaded('./dijkstra.pl').
:- ensure_loaded('./astar.pl').
:- ensure_loaded('./bestFirst.pl').
:- ensure_loaded('./hillClimbing.pl').

main :-
    current_prolog_flag(argv, Argv),
    (   Argv = [N_s, K_s, P_s, Pairs_s | _]
    ->  atom_number(N_s, N),
        atom_number(K_s, K),
        atom_number(P_s, P),
        read_term_from_codes(Pairs_s, Pairs, []),
        is_list(Pairs),
        (   Argv = [_, _, _, _, Algo_atom]
        ->  run_benchmark(N, K, P, Pairs, [Algo_atom])
        ;   run_benchmark(N, K, P, Pairs, [bfs, dfs, dijkstra, astar, best_first, hill_climb])
        )
    ;   format(stderr, "Usage: swipl -q -f benchmark.pl -- N K P Pairs [Algo]~n", []),
        halt(1)
    ).

run_benchmark(N, K, P, Pairs, Algos) :-
    % Deterministic small-world network (seed fixed inside graph.pl).
    gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
    maplist(run_algorithm(Vertices, Arestas, Pairs), Algos, PerAlgo),
    length(Pairs, PairCount),
    format('{"schema_version":"1.0","schema":"graph-search-benchmark-v1",', []),
    format('"graph":{"model":"watts-strogatz","n":~w,"k":~w,"p":~w,"seed":31},', [N, K, P]),
    format('"pair_count":~w,', [PairCount]),
    format('"pairs":', []),
    write_json_value(Pairs),
    format(',"algorithms":{', []),
    write_algo_entries(PerAlgo),
    format('}}', []),
    nl.

run_algorithm(Vertices, Arestas, Pairs, Algo, algo(Algo, Runs)) :-
    length(Pairs, NP),
    numlist(1, NP, Idxs),
    maplist(run_pair(Vertices, Arestas, Algo), Idxs, Pairs, Runs).

run_pair(Vertices, Arestas, Algo, Idx, Pair, run(Idx, S, G, Found, Dist, Path, Hops, TimeMs, Status)) :-
    Pair = [S, G],
    Goal =.. [run_search, Algo, S, G, Vertices, Arestas, Path, _Raw],
    timed_goal(Goal, TimeMs, Status),
    (   Status = ok, is_list(Path), Path = [_|_]
    ->  Found = true,
        path_distance(Path, Arestas, Dist),
        (   number(Dist) -> true ; Dist = null ),
        length(Path, Nv),
        Hops is Nv - 1
    ;   Found = false,
        Dist = null,
        Path = [],
        Hops = 0
    ).

run_search(bfs,       S, G, V, E, Path, _)      :- bfs(S, G, V, E, Path).
run_search(dfs,       S, G, V, E, Path, _)      :- dfs(S, G, V, E, Path).
run_search(dijkstra,  S, G, V, E, Path, D)      :- dijkstra(S, G, V, E, Path, D).
run_search(astar,     S, G, V, E, Path, D)      :- astar(S, G, V, E, Path, D).
run_search(best_first,S, G, V, E, Path, D)      :- best_first(S, G, V, E, Path, D).
run_search(hill_climb,S, G, V, E, Path, D)      :- hill_climb(S, G, V, E, Path, D).

% Wall-clock time around the search call only (graph creation and VM startup
% are excluded). ok = path found, not_found = search terminated without path,
% error = exception during the search.
timed_goal(Goal, TimeMs, Status) :-
    get_time(T0),
    catch((   call(Goal)
        ->  get_time(T1), TimeMs is (T1 - T0) * 1000, Status = ok
        ;   get_time(T1), TimeMs is (T1 - T0) * 1000, Status = not_found
        ), _E, (TimeMs = 0.0, Status = error)).

% Sum of edge weights along the returned path (uniform "distance" across algos).
path_distance([_], _, 0.0).
path_distance([A, B | Rs], Arestas, D) :-
    (   member(e(A, B, W), Arestas)
    ;   member(e(B, A, W), Arestas)
    ),
    path_distance([B | Rs], Arestas, D1),
    D is W + D1.

write_algo_entries([]).
write_algo_entries([algo(Name, Runs) | Rest]) :-
    format('"~w":{"runs":', [Name]),
    write_runs(Runs),
    format('}', []),
    (   Rest = [] -> true
    ;   format(',', []),
        write_algo_entries(Rest)
    ).

write_runs([]) :- format('[]', []).
write_runs([First | Rest]) :-
    format('[', []),
    write_run(First),
    write_runs_rest(Rest),
    format(']', []).

write_runs_rest([]).
write_runs_rest([H | T]) :- format(',', []),
                            write_run(H),
                            write_runs_rest(T).

write_run(run(Idx, S, G, Found, Dist, Path, Hops, TimeMs, Status)) :-
    format('{"pair_index":~w,"start":~w,"goal":~w,"found":~w,"distance":', [Idx, S, G, Found]),
    write_json_value(Dist),
    format(',"path":', []),
    write_json_value(Path),
    format(',"hops":~w,"time_ms":~w,"status":"~w"}', [Hops, TimeMs, Status]),
    (   Rest = [] -> true
    ;   format(',', []),
        write_runs(Rest)
    ).

write_json_value(null)      :- format('null', []).
write_json_value(true)      :- format('true', []).
write_json_value(false)     :- format('false', []).
write_json_value(Atom)      :- atom(Atom), !, format('"~s"', [Atom]).
write_json_value(Number)    :- number(Number), !, format('~w', [Number]).
write_json_value(List)      :- is_list(List), !, write_json_list(List).

write_json_list([])         :- format('[]', []).
write_json_list([H | T])    :- format('[', []),
                               write_json_value(H),
                               write_json_list_rest(T),
                               format(']', []).

write_json_list_rest([]).
write_json_list_rest([H | T]) :- format(',', []),
                                 write_json_value(H),
                                 write_json_list_rest(T).
