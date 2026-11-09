:- ensure_loaded('dfs.pl').

best_first(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    best_first_busca(Vf,Vertices,Arestas,[0-(Vi-[Vi])],Caminho).

best_first_busca(Vf,_,_,[_-(Vf-CaminhoAtual)|_], Caminho):-
    reverse(CaminhoAtual,Caminho).

best_first_busca(Vf,Vertices,Arestas,[H-(Atual-CaminhoAtual)| DemaisCaminhos],Caminho):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados)
    best_first_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho).

adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        D-(X-[X|CaminhoAtual]),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Vf,X2,Y2),Vertices),
            distancia(v(X,X1,Y1),v(Vf,X2,Y2),D)
        ),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,S),
    keysort(S,CaminhosAtualizados).
