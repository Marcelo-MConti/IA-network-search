:- ensure_loaded("../graph.pl").
dfs(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    dfs_busca(Vf,Vertices,Arestas,[0-(Vi-[Vi])],Caminho,[Vi],Distancia).

% Caso em que chegamos ao vertice final
dfs_busca(Vf,_,_,[Dist-(Vf-CaminhoAtual)|_],Caminho,_,Dist) :-
    reverse(CaminhoAtual,Caminho), !.

dfs_busca(Vf,Vertices,Arestas,[Dist-(Atual-CaminhoAtual) | DemaisCaminhos],Caminho,Fechados,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já visitados (conjunto fechado) para manter busca limitada
    exclude(member_right(Fechados), L, Lfiltrado),
    adicionar_caminhos_dfs(Lfiltrado,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices),
    dfs_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados],Distancia).

% Responsável por encontrar os filhos da aresta Atual
filhos(Atual,Arestas,CaminhoAtual,L):-
    findall(
        X,
        (
            (% Todos os Vertices que se conectam com Atual
                member(e(X,Atual,_),Arestas);
                member(e(Atual,X,_),Arestas)
            ), 
            not(member(X,CaminhoAtual)) % Os vertices novos n podem estar no caminho
        ),
        L 
    ).

% member_right/2: X é membro da segunda lista (para exclude).
member_right(Lista, X) :- member(X, Lista).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_dfs(L,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices):-
    findall(
        NovaDist-(X-[X|CaminhoAtual]),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Atual,AtualX,AtualY),Vertices),
            distancia(v(Atual,AtualX,AtualY),v(X,X1,Y1),Add),
            NovaDist is Dist + Add
        ),
        CaminhosNovos
    ),
    append(CaminhosNovos,DemaisCaminhos,CaminhosAtualizados).
