dfs(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    dfs_busca(Vf,Arestas,[Vi-[Vi]],Caminho).

% Caso em que chegamos ao vertice final
dfs_busca(Vf,_,[Vf-CaminhoAtual|Resto],Caminho) :-
    reverse(CaminhoAtual,Caminho).

% Caso em que não encontramos nenhum caminho possivel
dfs_busca(Vf,_,[],Caminho):-
    fail.

dfs_busca(Vf,Arestas,[Atual-CaminhoAtual | DemaisCaminhos],Caminho):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    adicionar_caminhos_dfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados),
    dfs_busca(Vf,Arestas,CaminhosAtualizados,Caminho).

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

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_dfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        X-[X|CaminhoAtual],
        member(X,L),
        CaminhosNovos
    ),
    append(CaminhosNovos,DemaisCaminhos,CaminhosAtualizados).