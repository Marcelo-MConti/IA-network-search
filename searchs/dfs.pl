:- ensure_loaded("../graph.pl").
dfs(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    dfs_busca(Vf,Arestas,[Vi-[Vi]],Caminho,[Vi]).

% Caso em que chegamos ao vertice final
dfs_busca(Vf,_,[Vf-CaminhoAtual|_],Caminho,_) :-
    reverse(CaminhoAtual,Caminho), !.

% Caso em que não encontramos nenhum caminho possivel
dfs_busca(_,_,[],_Caminho,_):-
    fail.

dfs_busca(Vf,Arestas,[Atual-CaminhoAtual | DemaisCaminhos],Caminho,Fechados):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já visitados (conjunto fechado) para manter busca limitada
    exclude(member_right(Fechados), L, Lfiltrado),
    adicionar_caminhos_dfs(Lfiltrado,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados),
    dfs_busca(Vf,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados]).

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
adicionar_caminhos_dfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        X-[X|CaminhoAtual],
        member(X,L),
        CaminhosNovos
    ),
    append(CaminhosNovos,DemaisCaminhos,CaminhosAtualizados).
