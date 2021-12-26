%%----------------------------------------------------
%% @doc
%% Jump Point Search 寻路算法
%% A*支持非网格式寻路
%% JPS只支持网格寻路
%% JPS Plus不支持动态阻挡
%% @author Eric Wong
%% @end
%% Created : 2021-12-26 11:31 Sunday
%%----------------------------------------------------
-module(jump_point).
-export([search/2, search/3]).

-define(ABS(__FLOAT__), erlang:abs(__FLOAT__)).
-define(MIN(__N1__, __N2__), erlang:min(__N1__, __N2__)).
-define(MAX(__N1__, __N2__), erlang:max(__N1__, __N2__)).
-define(HYPOTENUSE(__N1__, __N2__), math:sqrt(__N1__ * __N1__ + __N2__ * __N2__)).

-define(WIDTH, 1).
-define(LENGTH, 1).
-define(HYPOTENUSE, ?HYPOTENUSE(?WIDTH, ?LENGTH)).

-define(FRONT, 1).
-define(BACK, 2).
-define(LEFT, 3).
-define(RIGHT, 4).
-define(F_LEFT, 5).
-define(B_LEFT, 6).
-define(F_RIGHT, 7).
-define(B_RIGHT, 8).
-define(STRAIGHT, [?FRONT, ?BACK, ?LEFT, ?RIGHT]).
-define(OBLIQUE, [?F_LEFT, ?B_LEFT, ?F_RIGHT, ?B_RIGHT]).
-define(DIRECTION, ?STRAIGHT ++ ?OBLIQUE).

-define(D(__N__), io:format("DEBUG[LINE:~w] ~w~n", [?LINE, __N__])).

-record(node, {
          %% 节点位置
          pos
          %% 节点前置位置
          ,parent
          %% 节点代价 {起点代价, 终点预计代价}
          ,cost
         }).

%%----------------------------------------------------
%% 配置接口
%%----------------------------------------------------
%% @doc 配置地图可行走区域 输入坐标 返回true|false
-spec walkable(Pos::{integer, integer}) -> boolean().
walkable({0, 0}) -> true;
walkable({0, 1}) -> true;
walkable({0, 2}) -> true;
walkable({1, 2}) -> true;
walkable({2, 0}) -> true;
walkable({2, 1}) -> true;
walkable({2, 2}) -> true;
walkable(_) -> false.

%%----------------------------------------------------
%% 外部接口
%%----------------------------------------------------
%% @doc 搜索入口
-spec search({pos_integer, pos_integer}, {pos_integer, pos_integer}) -> [{pos_integer, pos_integer}] | {false, Reason::bitstring()}.
search(Start = {_, _}, Goal = {_, _}) ->
    search(Start, Goal, fun walkable/1).

search(Start = {_, _}, Goal = {_, _}, WalkableFun) when is_function(WalkableFun, 1) ->
    case WalkableFun(Start) andalso WalkableFun(Goal) of
        true ->
            case do([#node{pos = Start, cost = calc_score(Start, Start, {0, 0}, Goal)}], [], Goal, WalkableFun) of
                FinalSet = [_ | _] ->
                    trace_back(FinalSet, Start);
                Err = {false, _} ->
                    Err
            end;
        false ->
            {false, <<"UNREACHABLE AREA">>}
    end.

%%----------------------------------------------------
%% 内部私有
%%----------------------------------------------------
%% 递归主流程
do([], _, _, _) -> {false, <<"UNREACHABLE AREA">>};
do(OpenSet = [#node{pos = {X, Y}} | _], CloseSet, {X, Y}, _) ->
    OpenSet ++ CloseSet;
do(OpenSet = [Node = #node{pos = Pos = {_, _}, parent = Ppos, cost = Cost} | T], CloseSet, Goal, WalkableFun) ->
    Set = OpenSet ++ CloseSet,
    Dirs = dir(Pos, Ppos, WalkableFun),
    JPs = search(Pos, Dirs, WalkableFun, Goal),
    AddNodes = [#node{pos = JPPos, parent = Pos, cost = calc_score(JPPos, Pos, Cost, Goal)} || JPPos = {_, _} <- JPs, not lists:keymember(JPPos, #node.pos, Set)],
    NOpenSet = lists:sort(fun node_sort/2, AddNodes ++ T),
    do(NOpenSet, [Node | CloseSet], Goal, WalkableFun).

-compile(in_line).
%% 判断跳点
is_jp(Goal, _, _, Goal) -> true;
is_jp({X, Y}, WalkableFun, ?FRONT, _) ->
    (not WalkableFun({X - 1, Y - 1}) andalso WalkableFun({X - 1, Y}))
    orelse
    (not WalkableFun({X + 1, Y - 1}) andalso WalkableFun({X + 1, Y}));
is_jp({X, Y}, WalkableFun, ?BACK, _) ->
    (not WalkableFun({X - 1, Y + 1}) andalso WalkableFun({X - 1, Y}))
    orelse
    (not WalkableFun({X + 1, Y + 1}) andalso WalkableFun({X + 1, Y}));
is_jp({X, Y}, WalkableFun, ?LEFT, _) ->
    (not WalkableFun({X + 1, Y - 1}) andalso WalkableFun({X, Y - 1}))
    orelse
    (not WalkableFun({X + 1, Y + 1}) andalso WalkableFun({X, Y + 1}));
is_jp({X, Y}, WalkableFun, ?RIGHT, _) ->
    (not WalkableFun({X - 1, Y - 1}) andalso WalkableFun({X, Y - 1}))
    orelse
    (not WalkableFun({X - 1, Y + 1}) andalso WalkableFun({X, Y + 1}));
is_jp(Pos, WalkableFun, ?F_LEFT, Goal) ->
    search(Pos, [?FRONT, ?LEFT], WalkableFun, Goal) =/= [];
is_jp(Pos, WalkableFun, ?F_RIGHT, Goal) ->
    search(Pos, [?FRONT, ?RIGHT], WalkableFun, Goal) =/= [];
is_jp(Pos, WalkableFun, ?B_LEFT, Goal) ->
    search(Pos, [?BACK, ?LEFT], WalkableFun, Goal) =/= [];
is_jp(Pos, WalkableFun, ?B_RIGHT, Goal) ->
    search(Pos, [?BACK, ?RIGHT], WalkableFun, Goal) =/= [].

%% 搜索跳点
search({_, _}, [], _, _) -> [];
search(Pos = {_, _}, Dirs, WalkableFun, Goal) ->
    lists:flatten([search_jp(Pos, WalkableFun, Dir, Goal) || Dir <- Dirs]).

%% 任意方向搜索
search_jp({X, Y}, WalkableFun, Dir, Goal) ->
    Pos = case Dir of
              ?LEFT     -> {X - 1, Y};
              ?RIGHT    -> {X + 1, Y};
              ?FRONT    -> {X, Y + 1};
              ?BACK     -> {X, Y - 1};
              ?F_LEFT   -> {X - 1, Y + 1}; 
              ?F_RIGHT  -> {X + 1, Y + 1};
              ?B_LEFT   -> {X - 1, Y - 1};
              ?B_RIGHT  -> {X + 1, Y - 1}
          end,
    case WalkableFun(Pos) of
        false ->
            [];
        true ->
            case is_jp(Pos, WalkableFun, Dir, Goal) of
                true ->
                    [Pos];
                false ->
                    search_jp(Pos, WalkableFun, Dir, Goal)
            end
    end.

%% 回溯流程
trace_back(Set, Start) ->
    trace_back(Set, [], Start).

trace_back([#node{pos = {X, Y}} | _], Acc, {X, Y}) ->
    io:format("JPS COST:~w~n", [erlang:length(Acc) + 1]),
    [{X, Y} | Acc];
trace_back(Set = [#node{pos = {X, Y}, parent = Parent} | _], Acc, Start) ->
    case lists:keytake(Parent, #node.pos, Set) of
        {value, PNode = #node{pos = Parent}, Others} ->
            L = case Parent of
                    {X, Py} ->
                        [_ | T] = [{X, Ly} || Ly <- seq(Py, Y)],
                        T;
                    {Px, Y} ->
                        [_ | T] = [{Lx, Y} || Lx <- seq(Px, X)],
                        T;
                    {Px, Py} when abs(Px - X) =:= abs(Py - Y) andalso Px < X andalso Py < Y ->
                        [_ | T] = [{Px + L, Py + L} || L <- seq(0, X - Px)],
                        T;
                    {Px, Py} when abs(Px - X) =:= abs(Py - Y) andalso Px > X andalso Py < Y ->
                        [_ | T] = [{Px - L, Py + L} || L <- seq(0, X - Px)],
                        T;
                    {Px, Py} when abs(Px - X) =:= abs(Py - Y) andalso Px < X andalso Py > Y ->
                        [_ | T]= [{Px + L, Py - L} || L <- seq(0, X - Px)],
                        T;
                    {Px, Py} when abs(Px - X) =:= abs(Py - Y) andalso Px > X andalso Py > Y ->
                        [_ | T] = [{Px - L, Py - L} || L <- seq(0, X - Px)],
                        T
                end,
            %% trace_back([PNode | Others], [{X, Y} | Acc], Start);
            trace_back([PNode | Others], L ++ Acc, Start);
        _ ->
            {false, <<"TRACE BACK DATA ERR">>}
    end.

seq(N, N) -> [N];
seq(N, M) when N > M -> lists:reverse(lists:seq(M, N));
seq(N, M) when N < M -> lists:seq(N, M).

-compile(in_line).
calc_score(Pos = {_, _}, Ppos = {_, _}, Pcost = {_, _}, Goal = {_, _}) ->
    {start_cost(Pos, Ppos, Pcost), heuristic(Pos, Goal)}.

node_sort(#node{cost = {X1, X2}}, #node{cost = {Y1, Y2}}) ->
    X1 + X2 < Y1 + Y2.

-compile(in_line).
%% 搜索方向
%% X
%% ↑
%% P
dir({X, Y}, {X, Py}, W) when Y > Py ->
    case {not W({X - 1, Y - 1}) andalso W({X - 1, Y}), not W({X + 1, Y - 1}) andalso W({X + 1, Y})} of
        {true, true} ->
            [?FRONT, ?LEFT, ?RIGHT, ?F_RIGHT, ?F_LEFT];
        {true, false} ->
            [?FRONT, ?LEFT, ?F_LEFT];
        {false, true} ->
            [?FRONT, ?RIGHT, ?F_RIGHT];
        {false, false} ->
            [?FRONT]
    end;
%% P
%% ↓
%% X
dir({X, Y}, {X, Py}, W) when Y < Py ->
    case {not W({X - 1, Y + 1}) andalso W({X - 1, Y}), not W({X + 1, Y + 1}) andalso W({X + 1, Y})} of
        {true, true} ->
            [?BACK, ?LEFT, ?RIGHT, ?B_RIGHT, ?B_LEFT];
        {true, false} ->
            [?BACK, ?LEFT, ?B_LEFT];
        {false, true} ->
            [?BACK, ?RIGHT, ?B_RIGHT];
        {false, false} ->
            [?BACK]
    end;
%% P → X
dir({X, Y}, {Px, Y}, W) when X > Px ->
    case {not W({X - 1, Y + 1}) andalso W({X, Y + 1}), not W({X - 1, Y - 1}) andalso W({X, Y - 1})} of
        {true, true} ->
            [?RIGHT, ?FRONT, ?BACK, ?F_RIGHT, ?B_RIGHT];
        {true, false} ->
            [?RIGHT, ?FRONT, ?F_RIGHT];
        {false, true} ->
            [?RIGHT, ?BACK, ?B_RIGHT];
        {false, false} ->
            [?RIGHT]
    end;
%% X ← P
dir({X, Y}, {Px, Y}, W) when X < Px ->
    case {not W({X + 1, Y + 1}) andalso W({X, Y + 1}), not W({X + 1, Y - 1}) andalso W({X, Y - 1})} of
        {true, true} ->
            [?LEFT, ?FRONT, ?BACK, ?F_LEFT, ?B_LEFT];
        {true, false} ->
            [?LEFT, ?FRONT, ?F_LEFT];
        {false, true} ->
            [?LEFT, ?BACK, ?B_LEFT];
        {false, false} ->
            [?LEFT]
    end;
dir({X, Y}, {Px, Py}, _) when X > Px andalso Y > Py -> [?RIGHT, ?FRONT, ?F_RIGHT];
dir({X, Y}, {Px, Py}, _) when X < Px andalso Y > Py -> [?LEFT, ?FRONT, ?F_LEFT];
dir({X, Y}, {Px, Py}, _) when X > Px andalso Y < Py -> [?RIGHT, ?BACK, ?B_RIGHT];
dir({X, Y}, {Px, Py}, _) when X < Px andalso Y < Py -> [?LEFT, ?BACK, ?B_LEFT];
dir(_, undefined, _) -> ?STRAIGHT ++ ?OBLIQUE.

%%----------------------------------------------------
%% 代价函数
%% 评估从起点到该点的真实代价
%%----------------------------------------------------
-compile(in_line).
start_cost({X, Y}, {X, Py}, {PCost, _}) -> PCost + ?ABS(Py - Y) * ?WIDTH;
start_cost({X, Y}, {Px, Y}, {PCost, _}) -> PCost + ?ABS(Px - X) * ?LENGTH;
start_cost({X, _}, {Px, _}, {PCost, _}) -> PCost + ?ABS(Px - X) * ?HYPOTENUSE.

%%----------------------------------------------------
%% 评价函数
%% 评估从该点到终点的预计代价
%%
%% 若在网格中只能横纵移动采用曼哈顿距离
%% 若在网格中可以横纵对角移动采用对角距离
%% 若在网格中允许任意方向移动采用欧几里得距离
%% 此处选择对角距离
%%----------------------------------------------------
-compile(in_line).
%% 对角距离计算方法
heuristic({X, Y}, {Gx, Gy}) ->
    Dx = ?ABS(X - Gx),
    Dy = ?ABS(Y - Gy),
    Dx + Dy + (?HYPOTENUSE - 2) * ?MIN(Dx, Dy).

%% 曼哈顿距离计算方法
%% heuristic({X, Y}, {Gx, Gy}) ->
%%     Dx = ?ABS(X - Gx),
%%     Dy = ?ABS(Y - Gy),
%%     Dx + Dy.

%% 欧几里得距离计算方法
%% heuristic({X, Y}, {Gx, Gy}) ->
%%     Dx = ?ABS(X - Gx),
%%     Dy = ?ABS(Y - Gy),
%%     ?HYPOTENUSE(Dx, Dy).

%%----------------------------------------------------
%% 测试用例
%%----------------------------------------------------
-include_lib("eunit/include/eunit.hrl").
-ifdef(TEST).
search_test() ->
    [{0,0},{0,1},{0,2},{1,2},{2,2},{2,1},{2,0}] = search({0,0}, {2,0}).
-endif.
