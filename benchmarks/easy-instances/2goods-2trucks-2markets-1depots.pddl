(define (problem TPP)
(:domain TPP-Propositional)
(:objects
	goods1 goods2 - goods
	truck1 truck2 - truck
	market1 market2 - market
	depot1 - depot
	level0 level1 - level)

(:init
	(next level1 level0)
	(ready-to-load goods1 market1 level0)
	(ready-to-load goods1 market2 level0)
	(ready-to-load goods2 market1 level0)
	(ready-to-load goods2 market2 level0)
	(stored goods1 level0)
	(stored goods2 level0)
	(loaded goods1 truck1 level0)
	(loaded goods1 truck2 level0)
	(loaded goods2 truck1 level0)
	(loaded goods2 truck2 level0)
	(connected market1 market2)
	(connected market2 market1)
	(connected depot1 market2)
	(connected market2 depot1)
	(on-sale goods1 market1 level1)
	(on-sale goods2 market1 level1)
	(on-sale goods1 market2 level0)
	(on-sale goods2 market2 level0)
	(at truck1 depot1)
	(at truck2 depot1))

(:goal (and
	(stored goods1 level1)
	(stored goods2 level1)
    ))

)
