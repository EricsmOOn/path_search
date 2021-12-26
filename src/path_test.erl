%%----------------------------------------------------
%% @doc
%% path_test 寻路算法测试模块
%% 主要用于随机生成可行走区域 并可视化寻路结果
%% @author Eric Wong
%% @end
%% Created : 2021-12-26 00:16 Sunday
%%----------------------------------------------------
-module(path_test).
-export([t/3
         ,t/2
         ,gen_map/2
         ,print_map/0
         ,print_map/1
         ,walkable/1
        ]).

-define(ERR_RTN(__MSG__, __ARGS__), {false, erlang:list_to_bitstring(io_lib:format(__MSG__, __ARGS__))}).
-define(ERR(__MSG__), io:format(__MSG__)).
-define(ERR(__MSG__, __ARGS__), io:format(__MSG__, __ARGS__)).

%%----------------------------------------------------
%% 外部接口
%%----------------------------------------------------
t(Width, Length) ->
    gen_map(Width, Length),
    print_map(),
    AStarPath = a_star:search({0,0}, {Width - 1, Length - 1}, fun walkable/1),
    JPSPath = jump_point:search({0,0}, {Width - 1, Length - 1}, fun walkable/1),
    {io:format("A*:~n"), print_map(AStarPath), io:format("JPS:~n"), print_map(JPSPath)}.

t(SearchFun, Width, Length) when is_function(SearchFun, 3) andalso Width > 0 andalso Length > 0 ->
    gen_map(Width, Length),
    print_map(),
    case SearchFun({0,0}, {Width - 1, Length - 1}, fun walkable/1) of
        Path = [_ | _] ->
            print_map(Path);
        _ ->
            ?ERR("NO WAY")
    end.

gen_map(X, Y) when X > 0 andalso Y > 0 ->
    TMap = do_gen_map(X, Y),
    case lists:member({0,0}, TMap) andalso lists:member({X-1, Y-1}, TMap) of
        true ->
            put(test_map, {{X, Y}, TMap});
        false ->
            gen_map(X, Y)
    end;
gen_map(X, Y) -> ?ERR_RTN("ERROR INPUT ARGS X:~w Y:~w", [X, Y]).

walkable(Pos = {_, _}) ->
    case catch get(test_map) of
        {{_, _}, Map = [_ | _]} ->
            lists:member(Pos, Map);
        _ ->
            ?ERR("MAP NOT READY"),
            false
    end.

%%----------------------------------------------------
%% 内部私有
%%----------------------------------------------------
do_gen_map(X, Y) ->
    [{Px, Py} || Px <- lists:seq(0, X - 1), Py <- lists:seq(0, Y - 1), rand:uniform() > 0.3].

print_map() ->
    print_map([]).

print_map(Path = [_ | _]) ->
    case get(test_map) of
        {{Width, Length}, Map = [_ | _]} ->
            [io:format("_") || _ <- lists:seq(0, Width + 1)],
            io:format("~n"),
            [print_map_line(Width, Y, Map, Path) || Y <- lists:seq(0, Length - 1)],
            [io:format("^") || _ <- lists:seq(0, Width + 1)],
            io:format("~n");
        _ ->
            ?ERR("MAP NOT READY")
    end,
    ok;
print_map(Err) ->
    Err.

print_map_line(Width, Y, Walkable, Path) ->
    io:format("|"),
    [print({X, Y}, Walkable, Path) || X <- lists:seq(0, Width - 1)],
    io:format("|~n").

print(Pos = {_, _}, Walkable, Path) ->
    case lists:member(Pos, Walkable) of
        true ->
            case lists:member(Pos, Path) of
                true ->
                    io:format("+");
                false ->
                    io:format(" ")
            end;
        false ->
            io:format("*")
    end.
