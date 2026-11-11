:- ensure_loaded('dfs.pl').

bfs(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    bfs_busca(Vf,Vertices,Arestas,[0-(Vi-[Vi])],Caminho,[Vi],[Vi],Distancia).

% Caso em que chegamos ao vertice final
bfs_busca(Vf,_,_,[Dist-(Vf-CaminhoAtual)|_],Caminho,_,_,Dist) :-
    reverse(CaminhoAtual,Caminho), !.

bfs_busca(Vf,Vertices,Arestas,[Dist-(Atual-CaminhoAtual) | DemaisCaminhos],Caminho,Fechados,Visitados,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já expandidos ou já enfileirados (evita duplicatas na fila)
    exclude(member_right(Fechados), L, L1),
    exclude(member_right(Visitados), L1, Lfiltrado),
    adicionar_caminhos_bfs(Lfiltrado,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices),
    % registra os novos enfileirados para não reenfileirá-los depois
    findall(X, member(X, Lfiltrado), NovosVisitados),
    append(NovosVisitados, Visitados, VisitadosNovo),
    bfs_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados],VisitadosNovo,Distancia).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_bfs(L,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices):-
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
    append(DemaisCaminhos,CaminhosNovos,CaminhosAtualizados).