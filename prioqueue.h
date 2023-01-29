#include <set>

// Eintrag einer Vorrangwarteschlange, bestehend aus einer PrioritÃ¤t
// prio mit Typ P und zusÃ¤tzlichen Daten data mit Typ D.
// Die PrioritÃ¤t prio darf nicht direkt, sondern nur durch Aufruf
// von changePrio verÃ¤ndert werden.
// (Aus bestimmten GrÃ¼nden ist es praktischer, dass Entry global und
// nicht innerhalb von PrioQueue definiert wird.)
template <typename P, typename D>
struct Entry {
    P prio;
    D data;

    // Initialisierung mit PrioritÃ¤t p und zusÃ¤tzlichen Daten d.
    Entry (P p, D d) : prio(p), data(d) {}
};

// Minimum-Vorrangwarteschlange mit PrioritÃ¤ten des Typs P
// und zusÃ¤tzlichen Daten des Typs D.
// An der Stelle, an der PrioQueue fÃ¼r einen bestimmten Typ P verwendet
// wird, muss ein Kleiner-Operator (<) fÃ¼r den Typ P bekannt sein.
// Andere Vergleichsoperatoren (<=, >, >=, ==, !=) werden nicht benÃ¶tigt.
template <typename P, typename D>
struct PrioQueue {
    using Entry = ::Entry<P, D>;

    struct LessThan {
	bool operator() (Entry* e1, Entry* e2) const {
	    if (e1->prio < e2->prio) return true;
	    if (e2->prio < e1->prio) return false;
	    return e1 < e2;
	}
    };

    std::set<Entry*, LessThan> entries;

    // Ist die Warteschlange momentan leer?
    bool isEmpty () {
	return entries.empty();
    }

    // Neuen Eintrag mit PrioritÃ¤t p und zusÃ¤tzlichen Daten d erzeugen,
    // zur Warteschlange hinzufÃ¼gen und zurÃ¼ckliefern.
    // (Von insert erzeugte EintrÃ¤ge mÃ¼ssen vom Anwender freigegeben
    // werden, nachdem sie mit extractMinimum oder remove aus der
    // Warteschlange entfernt wurden und nicht mehr gebraucht werden.)
    Entry* insert (P p, D d) {
	Entry* e = new Entry(p, d);
	entries.insert(e);
	return e;
    }

    // Eintrag mit minimaler PrioritÃ¤t liefern.
    // (Nullzeiger bei einer leeren Warteschlange.)
    Entry* minimum () {
	if (entries.empty()) return nullptr;
	return *entries.begin();
    }

    // Eintrag mit minimaler PrioritÃ¤t liefern
    // und aus der Warteschlange entfernen (aber nicht freigeben).
    // (Bei einer leeren Halde wirkungslos mit Nullzeiger als Resultatwert.)
    Entry* extractMinimum () {
	Entry* e = minimum();
	if (e) entries.erase(entries.begin());
	return e;
    }

    // EnthÃ¤lt die Warteschlange den Eintrag e?
    // (Resultatwert false, wenn e ein Nullzeiger ist.)
    bool contains (Entry* e) {
	return e && entries.count(e);
    }

    // Eintrag e aus der Warteschlange entfernen (aber nicht freigeben).
    // (Wirkungslos mit Resultatwert false, wenn e ein Nullzeiger ist
    // oder e nicht zur aktuellen Warteschlange gehÃ¶rt.)
    bool remove (Entry* e) {
	return entries.erase(e);
    }

    // PrioritÃ¤t des Eintrags e auf p Ã¤ndern.
    // (Wirkungslos mit Resultatwert false, wenn e ein Nullzeiger ist
    // oder e nicht zur aktuellen Warteschlange gehÃ¶rt.)
    bool changePrio (Entry* e, P p) {
	if (!remove(e)) return false;
	e->prio = p;
	entries.insert(e);
	return true;
    }
};